# Symulacja Robota Mobilnego w ROS 2

Niniejsze repozytorium zawiera kod źródłowy opracowany w ramach pracy inżynierskiej. Projekt realizuje kompletną symulację robota mobilnego, obsługę sensorów (LiDAR 2D, odometria) oraz implementację i porównanie algorytmów SLAM (Slam Toolbox, Google Cartographer).

## Wymagania systemowe

Symulację stworzono i uruchamiano z wykorzystaniem następujących systemów:

* **System operacyjny:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Środowisko:** ROS 2 Humble Hawksbill
* **Symulator:** Ignition Gazebo 6 (Fortress)

## Instalacja i budowanie
### 1. Ściągnięcie repozytorium do workspace'a ROS2
### 2. Pobranie odpowiednich pakietów 
Przy próbie zbudowania terminal podpowie czego brakuje.
### 3. Zbudowanie projektu 
```colcon build --packages-select description_slambot ```


### 3. Uruchomienie symulacji 
```ros2 launch description_slambot main_launch.py ```


