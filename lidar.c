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
#include <pthread.h>
#include <assert.h>
#include <stdlib.h>

//첫번쨰 해결방안 여기에 있는 코드 다 처음부터 해. rplidar sdk c++ 로 만들어진걸 c언어로 바꿔서 메인에서 호출해서 사용.
// 두번째 해결방안 rplidar sdk 를 보고 분석해서 내 코드대로 바꾼다. 
//16000개의 패킷이 쌓였을때 한 프레임이 된다. 16000개의 데이터를 받은 뒤 pop 하는걸 만들면 될 것 같다. 매번 다름
// 어쩔때는 더 낮고 어쩔때는 더 많아. 항상 16000개가 안나오네 일단 이것도 문제임.
//함수에 메개변수로 넣어서 실행할때 값이 온전치 못하게 전달되는 현상을 목격했다.

//1초 동안의 데이터 패킷이 대략 16000개 찍히는걸 확인함. 대략 16000개지 전후로 숫자가 변함.
// 1초동안의 대략 10번정도 회전하니 이 데이터를 로깅해서 비교해보기. 오늘까지 할것 
// response Descriptor 가 매번 똑같이 오지 않아. 그니까 프로토콜에 명시한것과는 다르다. 이거는 타이밍 문제라고 함 해결하기 어려움. 계속 하다보면 어느순간에는 잘나옴.
// 이게 되면 62.5 us 마다 데이터를 read 해주고 처리해주는 반복문 만들기.
#define _GNU_SOURCE
#include <sched.h>

#define BAUDRATE B1000000
#define DEVICE "/dev/ttyUSB0"
//애초에 데이터 값이 잘못된거같음. 그전 값이랑 차이가 너무 심해 차이가 나면 안돼는데;
//데이터의 무결성을 체크 해봐야함. 내가 받아오는 데이터가 정말 잘 받아오고 있는게 맞는지
//데이터가 잘 안들어감
//일단 거리와 각도값을 csv 파일로 받아서 matlab 으로 시각화 시키기 .
//데이터를 받아서 읽는 과정에서 문제가 조금 있음. 

// void* thread_function(void* arg) {
//     cpu_set_t cpuset;
//     CPU_ZERO(&cpuset);  // CPU 집합 초기화
//     CPU_SET(0, &cpuset);  // CPU 0에만 실행되도록 설정

//     // 현재 스레드의 CPU 친화성 설정
//     if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
//         perror("sched_setaffinity 실패");
//         return NULL;
//     }

//     // 이제 이 스레드는 CPU 0에서만 실행됩니다
//     printf("스레드는 CPU 0에서 실행 중입니다\n");

//     return NULL;
// }
int fd = 0;
// 
rpLidar_t lidar;  // rpLidar_t 구조체 인스턴스 생
// void* thread_function(void* arg) {
//     cpu_set_t cpuset;
//     CPU_ZERO(&cpuset);  // CPU 집합 초기화
//     CPU_SET(0, &cpuset);  // CPU 0에만 실행되도록 설정

//     // 현재 스레드의 CPU 친화성 설정
//     if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
//         perror("sched_setaffinity 실패");
//         return NULL;
//     }

//     // 이제 이 스레드는 CPU 0에서만 실행됩니다
//     printf("스레드는 CPU 0에서 실행 중입니다\n");

//     return NULL;
// }
float rpLidar_CalcAngle(uint8_t _lowByte,uint8_t _highByte)
{
	uint16_t winkel=_lowByte<<7;
	winkel|=_highByte>>1;
	return winkel/64.0;
}

unsigned long millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);  // 초를 밀리초로 변환
}
float rpLidar_calcDistance(uint8_t _lowByte,uint8_t _highByte)
{   
    // printf("low byte = %u\n",_lowByte);
    // printf("처음 distance = %u\n",_highByte);
	uint16_t distance=(_lowByte)<<8;
    // printf("계산전 distance = %u\n",distance);
	distance|=_highByte;
    // printf("계산후 distance = %u\n",distance);

    // printf("raw distance = %u\n",distance);
	return distance/4.0f/1000;
	
}
//배열을 하나 만든뒤에 그 배열 안에 있는 값과 point 구조체 안에 있는 값이 같은지 확인하면되지. 같으면 문제는 없음.
void* rpLidar_awaitStandardScan(void* args1) {
    int fd1 = *((int *)args1);  // fd1을 매개변수로 받아옴

	stScanDataPoint_t point;
      unsigned long startTime = millis();  // 시작 시간 기록.
    uint8_t *pBuff=(uint8_t*)&lidar.DataBuffer; //Pointer to Buffer
    uint16_t count=0;
    int a = 0 ;
	bool frameStart=false;
    //1초에 데이터페킷을 16000개 보낸다. 그걸 확인해보자.
			//search for frameStart
				// new Frame? S=1 !S=0 an checkbit information can be found in protocol datasheet
           while(millis()<(startTime+1000)) 
	{     
        // a++;

          ssize_t bytesRead =  read(fd1,(uint8_t*)&point,5); //Read 가 문제인거 같은데..? 읽어들이는 데이터가 문제가 있으니까 이런 문제가 발생하는게 아닐까.?
    // if (bytesRead ==5) {

                    printf("quality : %02X\n" , point.quality);
                    printf("angle_low: %02X\n", point.angle_low);  // 초기화되지 않아 임의의 값 출력
                    printf("angle_high: %02X\n", point.angle_high);  // 초기화되지 않아 임의의 값 출력
                    printf("distance_low: %02X\n", point.distance_low);  // 초기화되지 않아 임의의 값 출력
                    printf("distance_high: %02X\n", point.distance_high);  // 초기화되지 않아 임의의 값 출력
                    // float D = rpLidar_calcDistance(point.distance_low , point.distance_high);
                    // float angle = rpLidar_CalcAngle(point.angle_low ,point.angle_high);
                    // printf("D = %f\n", D);
                    // printf("angle = %f\n", angle);

                    // printf("size = %d",sizeof(lidar.DataBuffer)/sizeof(point));
    
			if((point.quality&0x01)&&(!(point.quality&0x02))&&!frameStart) //  framestart? S=1 !S=0
			{
				if(point.angle_low&0x01) //check Bit
				{
					frameStart=true;
				}
			}


			else if(frameStart&&(point.quality&0x01)&&!(point.quality&0x02)&&count>1) //2. framestart?
			{
				if(point.angle_low&0x01) //11/25 01:33 분 수정함 원래 point.angle_high&0x01 이었음 하위와 상위 프로토콜이 바뀐다는 가정하에 수정함.
				{
                    // printf("qqqqqqqq\n");
					// uint16_t* result = malloc(sizeof(uint16_t)); // 동적 메모리 할당
                    // *result = count;
                    // return (void*)result;
				}
			}
            else if (frameStart) {
				memmove(pBuff,(uint8_t*)&point,sizeof(point)); //copy memory from incoming buffer to DataBuffer

				count++; //new point

   

				if(count<sizeof(lidar.DataBuffer)/sizeof(point))//inside the array bondaries?
				{
					pBuff=pBuff+5; //move pointer to next measure point in storage
				}            
                }

// }
    
    
    printf("카운트다운. = %u\n",count);
    }

    uint16_t* result = malloc(sizeof(uint16_t)); // 동적 메모리 할당
    *result = count;
    return (void*)result;
    
}
rp_descriptor_t resp_descriptor[]={{0xA5,0x5A,0x54,0x00,0x00,0x40,0x82},//Legacy Version
								{0xA5,0x5A,0x84,0x00,0x00,0x40,0x84},//Extended Version
								{0xA5,0x5A,0x54,0x00,0x00,0x40,0x85},//Dense Version
								{0xA5,0x5A,0x05,0x00,0x00,0x40,0x81},//StartScan
								{0xA5,0x5A,0x05,0x00,0x00,0x40,0x81},//Force Scan
								{0xA5,0x5A,0x14,0x00,0x00,0x00,0x04},//Get Device Info
								{0xA5,0x5A,0x03,0x00,0x00,0x00,0x06},//Get Health Info
								{0xA5,0x5A,0x04,0x00,0x00,0x00,0x15},//Get sample rate
								{0xA5,0x5A,0x04,0x00,0x00,0x00,0x15}};//Device configuration

rq_message_t req_message[]={{0xA5,0x25}, //Stop
							{0xA5,0x40},	//Reset
							{0xA5,0x20},//Scan
							{0xA5,0x82}, //Express scan
							{0xA5,0x21}, //Force Scan
							{0xA5,0x50}, //Get Info
							{0xA5,0x52}, //Get Health
							{0xA5,0x59}, //Get Samplerate
							{0xA5,0x84}};//Get device Conf 
uint8_t motor_start[6] = {0xA5,0xF0,0x02,0x01,0x36,0xCE};   

int init_uart(const char *device) {
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
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
//  struct termios2 tio;

//     ioctl(fd, TCGETS2, &tio);
//     bzero(&tio, sizeof(struct termios2));

//     tio.c_cflag = BOTHER;
//     tio.c_cflag |= (CLOCAL | CREAD | CS8); //8 bit no hardware handshake

//     tio.c_cflag &= ~CSTOPB;   //1 stop bit
//     tio.c_cflag &= ~CRTSCTS;  //No CTS
//     tio.c_cflag &= ~PARENB;   //No Parity

// #ifdef CNEW_RTSCTS
//     tio.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
// #endif

//     tio.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control


//     tio.c_cc[VMIN] = 0;         //min chars to read
//     tio.c_cc[VTIME] = 0;        //time in 1/10th sec wait

//     tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//     // raw output mode   
//     tio.c_oflag &= ~OPOST;

//     tio.c_ispeed = BAUDRATE;
//     tio.c_ospeed = BAUDRATE;


//     ioctl(fd, TCSETS2, &tio);



//     tcflush(fd, TCIFLUSH);

//     if (fcntl(fd, F_SETFL, FNDELAY))
//     {
//         close();
//         return false;
//     }


//     _is_serial_opened = true;
//     _operation_aborted = false;

//     //Clear the DTR bit to let the motor spin
//     clearDTR();
//     do {
//         // create self pipeline for wait cancellation
//         if (pipe(_selfpipe) == -1) break;

//         int flags = fcntl(_selfpipe[0], F_GETFL);
//         if (flags == -1)
//             break;

//         flags |= O_NONBLOCK;                /* Make read end nonblocking */
//         if (fcntl(_selfpipe[0], F_SETFL, flags) == -1)
//             break;

//         flags = fcntl(_selfpipe[1], F_GETFL);
//         if (flags == -1)
//             break;

//         flags |= O_NONBLOCK;                /* Make write end nonblocking */
//         if (fcntl(_selfpipe[1], F_SETFL, flags) == -1)
//             break;

//     } while (0);

    return fd;
}

// 모터 시작 명령 전송 함수


void *scan_start(void *args) {

    // chogihwa();

        int fd1 = *((int *)args);  // fd1을 매개변수로 받아옴
printf("전송 데이터: %02X %02X\n", req_message[2][0], req_message[2][1]);
        ssize_t bytes_written = write(fd1, req_message[2], 2);
    if (bytes_written < 0) {
        perror("모터 시작 명령 전송 오류");
    } else {
        printf("모터 시작 명령 전송 완료 - RPM: 전송된 바이트 수: \n");
    }
    rp_descriptor_t descr;

ssize_t len = read(fd1,(uint8_t*)&descr,7);
printf("len = %zd\n",len);
if (len >0) {
for (int i = 0 ; i < 7 ; i ++) {

    printf("%02X\n",descr[i]);


}
}
return NULL;

//여기에다가 descriptor 이 맞는지 화긴하는 로직 추가해주기. 그리고 return false 해주면 response descriptor 이 잘 오는지 안오는지 어느정도 확인 가능.
}

void scan_stop() {

        ssize_t bytes_written = write(fd, req_message[0], 2);
    if (bytes_written < 0) {
        perror("stop명령 전송 오류");
    } else {
        printf("stop명령 전송 완료 \n");
    }


}
void chogihwa() {

        ssize_t bytes_written = write(fd, req_message[1], 2);
    if (bytes_written < 0) {
        perror("초기화명령 전송 오류");
    } else {
        printf("초기화명령 전송 완료 \n");
    }


}

int main() {
    printf("메인 프로그램 시작...\n");
    fd = init_uart(DEVICE);
    if (fd == -1) {
        return -1;
    }
    printf("fd = %d\n",fd);
    printf("UART 파일 디스크립3터: %d\n", fd);

// start_motor();
    //  FILE *csv_file = fopen("lidar_data.csv", "w");
    // if (csv_file == NULL) {
    //     perror("파일 열기 실패");
    //     return -1;
    // }
    // fprintf(csv_file, "Angle,Distance\n");
    // //start_scan() 함수가 실행되면 descr 을 출력하면 됨
//     pthread_t p1;
//     pthread_t p2;
// void* result;
//     int rc;
//     int rb;

//     rc = pthread_create(&p1 , NULL ,scan_start,(void *)&fd);
//     if (rc != 0) {
//     printf("스레드 생성 실패: scan_start\n");
// }
// printf("scan_start 스레드가 시작되었습니다.\n");
//     rc = pthread_join(p1, NULL);  // scan_start가 끝날 때까지 기다림

//     if (rc != 0 ) {
//         printf("스레드 종료 대기 실패 \n");
//     }

//     printf("scan_start 스레드가 종료되었습니다.\n");

//     rb = pthread_create(&p2 , NULL ,rpLidar_awaitStandardScan,(void*)&fd);
//     printf("rb = %d",rb);
//     rb = pthread_join(p2,&result);
// uint16_t count = *((uint16_t*)result);
// printf("스캔 종료 후 count 값: %u\n", count);
// printf("Databuffer SIze = %ld\n",sizeof(lidar.DataBuffer));
// double angle = 0;
// float distance = 0;

// for (int i = 0 ; i < count; i ++) {
//     // printf("DataBuffer[%d] = quality: %02X, angle_low: %02X, angle_high: %02X, distance_low: %02X, distance_high: %02X\n",
//     //        i,
//     //        lidar.DataBuffer[i].quality,
//     //        lidar.DataBuffer[i].angle_low,
//     //        lidar.DataBuffer[i].angle_high,
//     //        lidar.DataBuffer[i].distance_low,
//     //        lidar.DataBuffer[i].distance_high);

//         lidar.Data[i].distance =rpLidar_calcDistance(lidar.DataBuffer[i].distance_low , lidar.DataBuffer[i].distance_high);
//         distance = lidar.Data[i].distance;
//         lidar.Data[i].angle= rpLidar_CalcAngle(lidar.DataBuffer[i].angle_low , lidar.DataBuffer[i].angle_high);
//         fprintf(csv_file, "%.2f,%.2f\n", lidar.Data[i].angle, distance);

// }

// scan_start();
// scan_stop();
// chogihwa();

    // fclose(csv_file);
    // close(fd);
    return 0;
}

//문제점 발견 일단 frameStart 는 5개 저 함수가 실행될때마다 false 가 되는게 맞음
//그다음 frameStart 부분을 발견하게 되면 거기서 부터 또 배열에 다가 넣고. 그게 반복되는것임
