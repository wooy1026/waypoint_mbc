## 주요 입력값  
```heading_callback``` > ```self.yaw``` : 동쪽을 0도로 하고 시계반대방향으로 증가하는 0-360도. (북쪽 90도, 남쪽270도)  
```stanley_control_angle``` > ```map.yaw``` : self.yaw의 좌표계와 동일.  
```stanley_control_angle``` > ```dir``` : ```np.dot``` 벡터 계산을 통한 주행 방향 판별.  ( 양수면 0-94도, 0이면 90도, 음수면 96-180도 )
