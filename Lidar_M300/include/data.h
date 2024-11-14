#ifndef __DATA__
#define __DATA__

#include <stdint.h>
#include <sys/time.h>

//#define TRANS_BUF_SIZE 0x4000
//#define TRANS_MASK	(TRANS_BUF_SIZE-1)
#define TRANS_BLOCK	0x200

#define ENC_BOTTOM		0
#define ENC_TOP			1

#define MAX_POINT_BUF	0x1000
#define POINT_ID_MASK	(MAX_POINT_BUF-1)

#define MAX_TICK_HIST	0x400
#define TICK_ID_MASK	(MAX_TICK_HIST-1)

typedef struct {
	uint16_t code;
	uint16_t len;
	uint16_t idx;
	uint16_t pad;
	uint8_t data[TRANS_BLOCK];
} TransBuf;

typedef struct
{
  uint8_t Header;
  int16_t Accel_X;
  int16_t Accel_Y;
  int16_t Accel_Z;
  int16_t Gyro_X;
  int16_t Gyro_Y;
  int16_t Gyro_Z;
  int8_t T;
  uint16_t TS;
  //uint32_t timestamp[2];
  uint64_t timestamp;
}__attribute__((packed)) IIM42652_FIFO_PACKET_16_ST;


struct CloudPoint
{
    uint64_t timestamp;
	int x, y, z;
	uint8_t energy;
};


#define PACK_PREAMLE 0X484C
#define DRIFT_RD_PACK 0x4357
#define DRIFT_MAGIC 0xD81F1CA1

struct CmdHeader
{
	uint16_t sign;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
};

typedef struct
{
	double R[3][3];
	float K[3];
	float B[3];
	double Gyro[3];
} IMUDrift;

typedef struct {
	IMUDrift imu;
} Drifts;

typedef struct
{
	uint32_t code;
	union {
		Drifts drifts;
		uint32_t body[126];
	};
	uint32_t crc;
} DriftCalib;

typedef struct {
  uint32_t depth : 24;
  uint32_t theta_hi : 8;
  uint32_t theta_lo : 12;
  uint32_t phi : 20;
  uint8_t reflectivity;
  uint8_t tag;
}__attribute__((packed)) M300LidarSpherPoint;

#define M300_PAC_POINT 64	

typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;      /**< unit: 0.1 us */
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
	uint64_t timestamp;		
  //uint8_t data[1];             /**< Point cloud data. */
	M300LidarSpherPoint points[M300_PAC_POINT];
}__attribute__((packed)) M300LidarEthernetPacket;


#endif