# A Robust Drone Delivery System for Destination Changes


![drone_system](https://user-images.githubusercontent.com/90020395/215795944-529e8543-ee67-4f64-a800-79e9b147c22e.png)


1. 제안된 시스템은 배송을 요청한 사용자의 위치 변화를 GPS로부터 실시간으로 전송받고, 이를 바탕으로 드론의 전역 비행 경로 및 목적지를 실시간으로 갱신한다. 

2. 장애물과 드론 간의 깊이 정보, 드론의 위치정보, 사용자의 위치정보를 바탕으로 수정된 APF(ArtificialPotential Field) 알고리즘이 수행되고, 이를 통해 드론은 장애물을 회피하며 동시에 사용자에게 안전히 다가간다. 

3. 드론에 탑재된 카메라를 통해 얻은 영상 정보를 바탕으로 YOLOv5가 수행되고, 이를 통해 사용자의 신원이 파악된다. 제안된 시스템은 시뮬레이션 및 실제 환경에서 수행되었다.
