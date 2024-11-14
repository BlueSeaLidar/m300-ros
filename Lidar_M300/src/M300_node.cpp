#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Imu.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include "data.h"
#include <stdio.h>
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <math.h>
#include <thread>
#include<mutex>
#include "Lidar_M300/CustomMsg.h"
#include "Lidar_M300/CustomPoint.h"
#include <queue>

IMUDrift m_imu_drift;

char m_lidar_addr[32];
std::string m_lidar_addr_string;
std::string m300_msg_frame_id;
std::string pointcloud2_frame_id;

int m_lidar_port = 6543;

pcl::PointCloud<pcl::PointXYZI> cloud_data;
std::vector<Lidar_M300::CustomPoint> cloud_data2;
std::queue<IIM42652_FIFO_PACKET_16_ST> imu_data;
__time_t start_sec;
__suseconds_t start_nsec;
int start_base_time;
ros::Publisher pcl_pub ;
ros::Publisher pcl_pub2;
ros::Publisher imu_pub; 
sensor_msgs::PointCloud2 output;
CloudPoint* tmp_cloud =  new CloudPoint[10000]();
std::mutex g_mutex;
std::mutex g_mutex2;
sensor_msgs::Imu imu;
bool use_imu;

int64_t imu_idx = 0;

void PubIMU(const uint8_t* buf)
{
	const IIM42652_FIFO_PACKET_16_ST* imu 
		= (IIM42652_FIFO_PACKET_16_ST*)buf;
	IIM42652_FIFO_PACKET_16_ST copy_of_imu_data = *imu;
	imu_data.push(copy_of_imu_data);
}

uint16_t Decode(uint16_t n, const uint8_t* buf)
{
	uint16_t idx = 0;
	int ndrop = 0;
	uint8_t drop[1024];
	while (idx < n)
	{ 
		uint8_t ch = buf[idx];

		if (ch == 0xd3) {
			if (idx + 25 > n) break;
			PubIMU(buf+idx+1);
			idx += 25;
		}
		else {
			drop[ndrop++] = ch;
			idx++;
		}
	}
	if (ndrop > 0) {
		printf("drop %d : %02x %02x %02x %02x %02x %02x\n",
			ndrop,
			drop[0], drop[1], drop[2],
			drop[3], drop[4], drop[5]);
	}

	return idx;
}

float errorFloat = 0.0;
void M3002Point(const M300LidarSpherPoint& m300, Lidar_M300::CustomPoint& point)
{
	point.reflectivity = m300.reflectivity;

	int32_t theta = m300.theta_hi;
	theta = (theta << 12) | m300.theta_lo;

	double ang = ( 90000 - theta) * M_PI / 180000;

	double depth = m300.depth / 1000.0;

	double r = depth * cos(ang);
	point.z = depth * sin(ang);

	
	ang = m300.phi * M_PI / 180000;
	point.x = cos(ang) * r;
	point.y = sin(ang) * r;


	if(point.x == errorFloat && point.y == errorFloat && point.z > errorFloat){
		ROS_INFO("error point");
	}

	point.line = 0;
	point.tag = 0;
}


void OnM300(const M300LidarEthernetPacket* m300)
{
	Lidar_M300::CustomPoint points[m300->dot_num];
	uint64_t timestamp[m300->dot_num];

	for (int i = 0; i < m300->dot_num; i++) 
	{
		M3002Point(m300->points[i], points[i]);
		timestamp[i] = m300->timestamp
			+ i * m300->time_interval * 100 
			/ (m300->dot_num-1); 
	}

	g_mutex.lock();
	{
		

		for (int i=0; i<m300->dot_num; i++)
		{
			// ROS_INFO("%ld",m300->dot_num);

		if (cloud_data2.size() == 0 )
		{
			
			start_sec = timestamp[0]/ 1000000000;
			start_nsec =  timestamp[0] % 1000000000;
			start_base_time = start_sec*1000000000 + start_nsec;
			 
		}
			points[i].offset_time = timestamp[i] - start_base_time;

			cloud_data2.push_back(points[i]);
			// ROS_INFO("%ld",timestamp[i]);

			//rviz
			if(points[i].reflectivity < 0){
				continue;
			}
			pcl::PointXYZI to_cloud;
			to_cloud.x = points[i].x;
			to_cloud.y = points[i].y;
			to_cloud.z = points[i].z;
			to_cloud.intensity= points[i].reflectivity;
			
			cloud_data.push_back(to_cloud);
		}
	}
	g_mutex.unlock();
}
void* UDPThreadProc()
{
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	int port_index = 6668;
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port_index);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	while (iResult != 0)
	{
		printf("bind 6668 failed!\n");
		addr.sin_port = htons(++port_index);
		iResult = ::bind(sock, (struct sockaddr*)&addr, sizeof(addr));
		//return 0;
	}
	printf("bind port success!\n");
	uint16_t wait_idx = 0;
	uint8_t m300_idx = 0;

	uint16_t nlen = 0;
	uint8_t buffer[TRANS_BLOCK*2];

	while (1)
	{
		uint8_t buf[1024];
		sockaddr_in addr;
		socklen_t sz = sizeof(addr);
		int dw = recvfrom(sock, (char*)buf, sizeof(buf), 0, 
			(struct sockaddr*)&addr, &sz);

		int m300_size = sizeof(M300LidarEthernetPacket);
		if (buf[0] == 0 && dw == m300_size)
		{
			const M300LidarEthernetPacket* m300 = (M300LidarEthernetPacket*)&buf;

			if (m300->udp_cnt != m300_idx)
			{
				printf("packet lost : %d != %d\n",
					m300->udp_cnt, m300_idx);
			}
			m300_idx = m300->udp_cnt+1;
			OnM300(m300);
		}
		else if (buf[0] == 0xfa && buf[1] == 0x88) 
		{
			const TransBuf* trans = (TransBuf*)buf;

			if (trans->idx != wait_idx) {
				printf("%d != %d\n", trans->idx, wait_idx);
			}

			memcpy(buffer + nlen, trans->data, trans->len);
			nlen += trans->len;
			wait_idx = trans->idx + 1;

			uint16_t n = Decode(nlen, buffer);
			for (int i = 0; i < nlen - n; i++) {
				buffer[i] = buffer[n + i];
			}
			nlen -= n;
		}
		else {
			const uint8_t* ptr = (uint8_t*)&buf;
			printf("packet len %d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
				dw,
				ptr[0], ptr[1], ptr[2], ptr[3],
				ptr[4], ptr[5], ptr[6], ptr[7]);
		}
	}

	close(sock);
	return 0;
}

void PubCloudThreadProc()
{ 
	while (1)
	{
	
	
	g_mutex.lock();
	{
	if(cloud_data.size() == cloud_data.height*cloud_data.width && cloud_data.size() !=0  && cloud_data.width !=0  && cloud_data.height !=0){	
	cloud_data.height=1;
	cloud_data.width=cloud_data.size();
	pcl::toROSMsg(cloud_data,output);
	output.header.stamp=ros::Time::now();
	output.header.frame_id=pointcloud2_frame_id;
	pcl_pub.publish(output);
	}
	cloud_data.clear();
	}

	{
	Lidar_M300::CustomMsg mymsg;
	mymsg.points = cloud_data2;
	mymsg.point_num = cloud_data2.size();
	mymsg.lidar_id = 192;
	mymsg.header.frame_id = m300_msg_frame_id;
	mymsg.header.stamp.sec = start_sec;
	mymsg.header.stamp.nsec = start_nsec;
	// mymsg.header.stamp = ros::Time().now();
	// ROS_INFO("cloudsec:%ld",mymsg.header.stamp.sec);
	mymsg.header.seq++;
	mymsg.rsvd = {0,0,0};
	mymsg.timebase = start_sec*1000000000 + start_nsec;
	pcl_pub2.publish(mymsg);
	cloud_data2.clear();
	}
	g_mutex.unlock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));	
	}
}

void PubImuThreadProc()
{
	while(1){
	if(!imu_data.empty()){
	double accel[3], gyro[3];

   	IIM42652_FIFO_PACKET_16_ST imu_stmp = imu_data.front();
	imu_data.pop();


	imu.angular_velocity.x = imu_stmp.Gyro_X * 4000.0 / 0x10000 * M_PI / 180 + m_imu_drift.Gyro[0];
	imu.angular_velocity.y = imu_stmp.Gyro_Y * 4000.0 / 0x10000 * M_PI / 180 + m_imu_drift.Gyro[1];
	imu.angular_velocity.z = imu_stmp.Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + m_imu_drift.Gyro[2];

	// double vx = (imu_stmp.Accel_X * 4.0 / 0x10000) * m_imu_drift.K[0] + m_imu_drift.B[0];
	// double vy = (imu_stmp.Accel_Y * 4.0 / 0x10000) * m_imu_drift.K[1] + m_imu_drift.B[1];
	// double vz = (imu_stmp.Accel_Z * 4.0 / 0x10000) * m_imu_drift.K[2] + m_imu_drift.B[2];

	double vx = (imu_stmp.Accel_X * 4.0 / 0x10000);
	double vy = (imu_stmp.Accel_Y * 4.0 / 0x10000);
	double vz = (imu_stmp.Accel_Z * 4.0 / 0x10000);

	imu.linear_acceleration.x= vx * m_imu_drift.R[0][0] + vy * m_imu_drift.R[0][1] + vz * m_imu_drift.R[0][2];
	imu.linear_acceleration.y= vx * m_imu_drift.R[1][0] + vy * m_imu_drift.R[1][1] + vz * m_imu_drift.R[1][2];
	imu.linear_acceleration.z= vx * m_imu_drift.R[2][0] + vy * m_imu_drift.R[2][1] + vz * m_imu_drift.R[2][2];


	uint64_t nanosec = imu_stmp.timestamp;
	imu.header.stamp.sec = nanosec/1000000000;
	imu.header.stamp.nsec = nanosec%1000000000;

	imu_pub.publish(imu);
	
	// std::this_thread::sleep_for(std::chrono::milliseconds(5));	
	}
	}
}


uint32_t stm32crc(const uint32_t* ptr, uint32_t len)
{
	uint32_t xbit, data;
	uint32_t crc32 = 0xFFFFFFFF;
	const uint32_t polynomial = 0x04c11db7;

	for (uint32_t i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (uint32_t bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}

int PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void* buf, uint8_t* netbuf)
{
	CmdHeader* hdr = (CmdHeader*)netbuf;

	hdr->sign = PACK_PREAMLE;
	hdr->cmd = type;// S_PACK;
	hdr->sn = sn;// rand();

	hdr->len = len;
	int len4 = ((len + 3) >> 2) * 4;
	if (len > 0) {
		memcpy(netbuf + sizeof(CmdHeader), buf, len);
	}

	int n = sizeof(CmdHeader);
	uint32_t* pcrc = (uint32_t*)(netbuf + sizeof(CmdHeader) + len4);
	pcrc[0] = stm32crc((uint32_t*)(netbuf + 0), len4 / 4 + 2);

	return len4 + 12;
}

int SendNetPack(int sock, uint16_t type, uint16_t len, const void* buf) 
{
	uint8_t netbuf[1024];
	int netlen = PackNetCmd(type, len, rand(), buf, netbuf);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(m_lidar_addr);
	addr.sin_port = htons(m_lidar_port);

	return sendto(sock, (char*)netbuf, netlen, 0,
	 (struct sockaddr*)&addr, sizeof(struct sockaddr));
}

int read_calib(int sockfd, const char* lidar_ip, int port)
{
	printf("read data from %s:%d\n", lidar_ip, port);
	uint8_t zbuf[16] = { 0 };
	SendNetPack(sockfd, DRIFT_RD_PACK, 4, zbuf);
	
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd, &readfds);

	struct timeval tv;
	tv.tv_sec = 3;
	tv.tv_usec = 0;
	int retval = select(sockfd + 1, &readfds, NULL, NULL, &tv);
	if (retval < 0) {
		printf("sock err\n");
		return retval; 
	}
	if (retval == 0) {
		printf("sock timeout\n");
		return -1;
	}

	char buf[1024];
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);
	int n = recvfrom(sockfd, (char*)&buf, sizeof(buf), 0, 
		(struct sockaddr*)&addr, &sz);

	const CmdHeader* hdr = (CmdHeader*)buf;
	if (hdr->sign != PACK_PREAMLE)
	{
		printf("unknown response\n");
		return -1;
	}

	uint32_t len4 = ((hdr->len + 3) >> 2) * 4;
	const uint32_t* pcrc = (uint32_t*)((char*)buf + sizeof(CmdHeader) + len4);
	uint32_t chk = stm32crc((uint32_t*)buf, len4 / 4 + 2);

	if (*pcrc != chk) {
		printf("crc error\n");
		return -1;
	}

	uint16_t type = ~(hdr->cmd);
	if (type != DRIFT_RD_PACK || hdr->len < sizeof(DriftCalib))
	{
		printf("not drift data\n");
		return -1;
	}

	DriftCalib drift;
	memcpy(&drift, buf+sizeof(CmdHeader), sizeof(drift));
	if (drift.code != DRIFT_MAGIC) {
		printf("bad drift data\n");
		return -1;
	}

	m_imu_drift = drift.drifts.imu;

	printf("imu : %f, %f, %f; %f, %f, %f; %f, %f, %f;", 
		m_imu_drift.R[0][0], m_imu_drift.R[0][1], m_imu_drift.R[0][2],
		m_imu_drift.R[1][0], m_imu_drift.R[1][1], m_imu_drift.R[1][2],
		m_imu_drift.R[2][0], m_imu_drift.R[2][1], m_imu_drift.R[2][2]);

	printf("gyro : %f, %f, %f\n", 
		m_imu_drift.Gyro[0], m_imu_drift.Gyro[1], m_imu_drift.Gyro[2]);

	printf("B-K : [B:%f, %f, %f K:%f, %f, %f], %d\n",
		m_imu_drift.B[0], m_imu_drift.B[1], m_imu_drift.B[2],m_imu_drift.K[0],
		m_imu_drift.K[1],m_imu_drift.K[2]);	

	return 0;
}


main (int argc, char **argv)
{
  ros::init(argc, argv, "Lidar_M300");
  ros::NodeHandle nh;
  nh.param("/M300/frame_id_PointCloud2", pointcloud2_frame_id, (std::string)"base_link");
  nh.param("/M300/frame_id_Msg", m300_msg_frame_id, (std::string)"m300_frame");
  nh.param("/M300/lidar_ip", m_lidar_addr_string, (std::string)"192.168.1.10");
  std::cout << m_lidar_addr_string << std::endl;
  strncpy(m_lidar_addr, m_lidar_addr_string.c_str(), sizeof(m_lidar_addr) - 1);
  m_lidar_addr[sizeof(m_lidar_addr) - 1] = '\0';


  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/M300_points", 10);
  pcl_pub2 = nh.advertise<Lidar_M300::CustomMsg>("/m300/lidar", 10);
  imu_pub = nh.advertise<sensor_msgs::Imu>("/m300/imu",10);
  imu.header.frame_id=m300_msg_frame_id;

	int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	while (read_calib(sockfd, m_lidar_addr, m_lidar_port) != 0);
	close(sockfd);


  
  // 创建新线程并运�?�UDPThreadProc函数
  std::thread udp_thread(UDPThreadProc);
  // 创建点云发布线程并运行PubCloudThreadProc函数
  std::thread pubcloud_thread(PubCloudThreadProc);
  std::thread pubimu_thread(PubImuThreadProc);
  
while (ros::ok())
{	
	// std::this_thread::sleep_for(std::chrono::milliseconds(100));	
}

  return 0;
}
