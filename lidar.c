#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <stdint.h>  // uint8_t와 같은 타입을 정의하기 위한 헤더 파일 추가
#include "rpLidar.h"

#include <sys/time.h>
#define BUFFER_SIZE 7  // 버퍼 크기 설정
#define BAUDRATE B1000000
#define DEVICE "/dev/ttyUSB0"
//애초에 데이터 값이 잘못된거같음. 그전 값이랑 차이가 너무 심해 차이가 나면 안돼는데;
//데이터의 무결성을 체크 해봐야함. 내가 받아오는 데이터가 정말 잘 받아오고 있는게 맞는지
int fd = 0;
// response_packet 오는거 확인 완료.
uint8_t motor_start_cmd[2] = {0xA5, 0x25}; //잘 안돼면 데이터 타입 바꿔보기.
uint8_t resp_descriptor[7] = {
    0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81
};     
rpLidar_t lidar;  // rpLidar_t 구조체 인스턴스 생

uint16_t rpLidar_ReadMeasurePoints() {

	uint16_t count=0;
		count =	rpLidar_awaitStandardScan();

	
	return count;

}
/*
 *  @author KKest
 *		@created 19.01.2022
 *	
 * Types for rpLidar library
 *
 */
float rpLidar_calcDistance(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t distance=(_highByte <<8);
    uint16_t distance1 =(_lowByte);
    int convert_highByte_i = (int)distance;
    int convert__lowByte_i = (int)distance1;
    int real_distance = (convert_highByte_i+convert__lowByte_i)/4;
    // float rd = (float)distance/4.0f;
	return real_distance;
}
// float rpLidar_CalcAngle(uint8_t _lowByte,uint8_t _highByte)
// {
// 	uint16_t winkel=_highByte<<7;
// 	winkel|=_lowByte>>1;
// 	return winkel/64.0;
// }



//애초에 이상한 값이 들어오는거 같은데.. 왜냐하면 이게 p.이걸로 계산했을때의 값이랑 DataBuffer 에 있는 값으로 계산을 하나 똑같으니.
// 측정된 포인트를 읽는 함수
unsigned long millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);  // 초를 밀리초로 변환
}
// // 표준 스캔을 대기하는 함수
uint16_t rpLidar_awaitStandardScan() {
    unsigned long startTime = millis();  // 시작 시간 기록.
	stScanDataPoint_t point;
        	uint8_t *pBuff=(uint8_t*)&lidar.DataBuffer; //Pointer to Buffer
            printf("Quality: %u\n", point.quality);  // 초기화되지 않아 임의의 값 출력
            printf("angle_low: %u\n", point.angle_low);  // 초기화되지 않아 임의의 값 출력
            printf("angle_high: %u\n", point.angle_high);  // 초기화되지 않아 임의의 값 출력
            printf("distance_low: %u\n", point.distance_low);  // 초기화되지 않아 임의의 값 출력
            printf("distance_high: %u\n", point.distance_high);  // 초기화되지 않아 임의의 값 출력

    uint16_t count=0;
	bool frameStart=false;
			//search for frameStart
				// new Frame? S=1 !S=0 an checkbit information can be found in protocol datasheet
           while(millis()<(startTime+5000)) //timeout after 5 seconds //5초동안 지속하지 않는거 같아. 
	{     
          
          ssize_t bytesRead =  read(fd,(uint8_t*)&point,sizeof(point));
        //   printf("byte_Read = %ld\n",bytesRead);
        //   if (bytesRead != sizeof(point)) {
        //     printf("읽기오류\n");
        //   }
        if (bytesRead != sizeof(point)) {
            // printf("읽기오류\n");
            continue;  // 오류 발생 시 다음 루프 진행
        }
			if((point.quality&0x01)&&(!(point.quality&0x02))&&!frameStart) //  framestart? S=1 !S=0
			{

				if(point.angle_high&0x01) //check Bit
				{
					frameStart=true;
                                // printf("frameStart set to true\n");

				}
			}


			else if(frameStart&&(point.quality&0x01)&&!(point.quality&0x02)&&count>1) //2. framestart?
			{
                // printf("ws\n");

				if(point.angle_high&0x01)
				{
            // printf("Returning count: %d\n", count); // 출력
					return count;
				}
			}


            if (frameStart) {
                // printf("sdf\n");
				memmove(pBuff,(uint8_t*)&point,sizeof(point)); //copy memory from incoming buffer to DataBuffer
				count++; //new point

				if(count<sizeof(lidar.DataBuffer)/sizeof(point))//inside the array bondaries?
				{
 
					pBuff=pBuff+5; //move pointer to next measure point in storage
				}            
                // break;
                }
		    // printf("frameStart = %d\n",frameStart);



}
	    return count;   
}

bool rpLidar_CompareDescriptor(uint8_t *descr1, uint8_t *descr2) {
	for(size_t i=0;i<7;i++)
	{
    printf("descr1 = %02X\n",descr1[i]);

		if( descr1[i]!= descr2[i])
		{
			return false;
		}
	}
	return true;
}
 //정지했다가 시작했을때 처음 주는게 A5 5A 05 00 00 40 81 그다음 부터는 각도와 거리에 대한 데이터를 주는것이 맞음. 그렇다면 문제는 계산하는 부분에서 문제가 있을 수 밖에 없다.
// RPLIDAR 모터 시작 명령 (A5 A8 02 RPM CHECKSUM)
//배열 한칸이 정말 한 바이트로 인식하는지 확인 필요 .

//write 가 보내기 
// UART 초기화 함수
int init_uart(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("UART 열기 실패");
        return -1;
    }

    // Non-blocking 모드 비활성화
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    // 보드레이트 설정
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, 9600);
    // 데이터 설정: 8N1 (8비트 데이터, 패리티 없음, 스톱 비트 1개)
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~(unsigned int)PARENB;
    options.c_cflag &= ~(unsigned int)CSTOPB;
    options.c_cflag &= ~(unsigned int)CSIZE;
    options.c_cflag |= CS8; //데이터 비트 크기 

    // 설정 적용
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// 모터 시작 명령 전송 함수
bool start_motor(int fd, int rpm) { //start

uint8_t descr[7] = {0,};

    ssize_t bytes_written = write(fd, motor_start_cmd, sizeof(motor_start_cmd));
    if (bytes_written < 0) {
        perror("모터 시작 명령 전송 오류");
    } else {
        printf("모터 시작 명령 전송 완료 - RPM: %d, 전송된 바이트 수: %zd\n", rpm, bytes_written);
    }
ssize_t len = read(fd,descr,sizeof(descr));
if (len >0) {
for (int i = 0 ; i < 7 ; i ++) {
    printf("drscr = %02X\n",descr[i]);
}
}
if (rpLidar_CompareDescriptor(descr,resp_descriptor)) {
    lidar.status=true;
    return true;
}
return false;
}




int main() {
    printf("메인 프로그램 시작...\n");
    
    fd = init_uart(DEVICE);
    if (fd == -1) {
        return -1;
    }

    printf("UART 파일 디스크립터: %d\n", fd);
    if(start_motor(fd,255)) {
        uint16_t count =  rpLidar_ReadMeasurePoints();
         printf("count = %hu\n",count);
          for(uint16_t i=0;i<count;i++) {
            int distance = rpLidar_calcDistance(lidar.DataBuffer[i].distance_low , lidar.DataBuffer[i].distance_high);
            printf("distance = %d\n",distance);
      

          }

    }
    // 모터 시작 및 RPM 설정
    // if (start_motor(fd, 255)) {
    //     printf("Sdfsdfasdfasdf\n");
// if (lidar.status) {
//    while(1) {

//           uint16_t count =  rpLidar_ReadMeasurePoints();
//         //   printf("count = %hu\n",count);
//         // printf("Quality: %02X, ", lidar.DataBuffer[count].quality);
//         printf("Angle Low: %02x, ", lidar.DataBuffer[0].angle_low);
//         printf("Angle High: %02X, ", lidar.DataBuffer[0].angle_high);
//         printf("Distance Low: %02X, ", lidar.DataBuffer[0].distance_low);
//         printf("Distance High: %02X\n", lidar.DataBuffer[0].distance_high);
//         //  for(uint16_t i=0;i<count-1;i++) {
//             float distance = rpLidar_calcDistance(lidar.DataBuffer[0].distance_low , lidar.DataBuffer[0].distance_high);
//     printf("distance = %.2f\n",distance);


//         //  }
//    }
// }
// else{
//     printf("다시\n");
// }
    // }
    // }
    // else {
    //     printf("what the fuck \n");
    // }

    // receive_response_and_data(fd);
    // UART 포트 닫기
    close(fd);
    return 0;
}

//문제점 발견 일단 frameStart 는 5개 저 함수가 실행될때마다 false 가 되는게 맞음
//그다음 frameStart 부분을 발견하게 되면 거기서 부터 또 배열에 다가 넣고. 그게 반복되는것임
