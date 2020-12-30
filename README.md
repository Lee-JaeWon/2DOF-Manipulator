# 작품 제목

광운대학교 로봇학부 학술소모임 **'BARAM'** 20년도 후반기 2DOF-Manipulator에 대한 소스코드입니다.  

## 개발 환경
|OS|사용 언어|사용 IDE|
|:---:|:---:|:---:|
| Windows | C | Atmel Studio |

## 프로젝트 개발 동기

-  의족이나 웨어러블 기기에 관심이 있어 관련 기초나 지식을 경험해보기 위하여 Manipulator로 작품 선정
-  하드웨어 3D 모델링(Inventor)
-  2차원 상에서의 역기구학 풀이(SCARA)
-  MCU에 대한 전반적인 이해도 증가

## 프로젝트 개요
1. Dynamixel AX시리즈 구동(AX-12W, AX-12+)_packet 통신    
2. 2차원에서의 역기구학 풀이 EE(End Effector)를 통한 각도 도출 코드 구현 
3. ATmega128의 2개의 UART를 이용하여 serial 통신 및 packet 통신  
4. serial 통신 _ 터미널 에서 좌표 입력시 모터 구동
> - 
> - 
## System Architecture
<p align="center"><img src=" " width="600px"></p>  


### Code Overview  
- 
- 
- 
- 

### Project scenario

1. x,y 좌표 및 위 아래 움직임 여부(1, 0)를 컴퓨터에 입력한다.
2. serial 통신을 통해 그 값을 Atmega128에서 공식을 통해 계산, 각 관절의 각도 값을 도출한다.
3. 모터가 도출된 각도에 맞게 적절히 움직인다.
4. 말단부의 집게를 통해 원하는 위치에 물체를 옮길 수 있도록 하는 동작을 수행한다.


## 프로젝트 결과

<p align="center"><img src="https://user-images.githubusercontent.com/72693388/103347630-ddd54680-4ada-11eb-8e54-b19efe314fb0.jpg" width="500px"></p>  
<p align="center"> 전체적인 작품의 모습 </p>  

<p align="center"><img src=" " width="500px"></p>  
<p align="center"> 사진2에 대한 설명 </p>  

<p align="center"><img src=" " width="500px"></p>  
<p align="center"> 사진3에 대한 설명 </p>
