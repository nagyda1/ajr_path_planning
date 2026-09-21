# AJR nagy féléves – A* és RRT alapú útvonaltervező

Autonóm Járművek és Robotok (AJR) tárgy nagy féléves beadandója. A projekt egy **optimális útvonaltervező rendszert** valósít meg autonóm jármű számára, statikus és dinamikus térkép alapján, A* és RRT/RRT* algoritmusokkal, ROS 2 Humble alatt, RViz2-es vizualizációval.

## Téma leírása

A feladat egy útvonaltervező rendszer kidolgozása, amely:
- statikus térkép (occupancy grid) alapján optimális útvonalat keres A* algoritmussal,
- mintavételezéses (RRT / RRT*) tervezést valósít meg jármű-kinematikai korlátok (kanyarodási sugár) figyelembevételével,
- dinamikus akadály esetén újratervezést végez,
- a két megközelítést (A* vs. RRT) irodalomkutatással és saját méréssel (futásidő, útvonalhossz, sikeresség) hasonlítja össze,
- kitekintést ad automatikus parkolási és ütközéselkerülési szcenáriókra.

## Csomópontok (nodes)

| Node | Bemenet (subscribe) | Kimenet (publish) | Leírás |
|---|---|---|---|
| `map_server_node` | – | `/map` (`nav_msgs/OccupancyGrid`) | Statikus/generált térkép publikálása |
| `astar_planner_node` | `/map`, `/goal_pose` | `/astar/path` (`nav_msgs/Path`), `/astar/markers` (`visualization_msgs/MarkerArray`) | A* keresés a rácstérképen |
| `rrt_planner_node` | `/map`, `/goal_pose` | `/rrt/path` (`nav_msgs/Path`), `/rrt/markers` (`visualization_msgs/MarkerArray`) | RRT/RRT* keresés kinematikai korlátokkal |
| `dynamic_obstacle_node` | – | `/dynamic_obstacle` (`visualization_msgs/Marker`) | Mozgó akadály szimulálása |
| `evaluator_node` | mindkét tervező kimenete | terminál / CSV log | A* és RRT teljesítmény-összehasonlítás |

## Telepítés és futtatás

Előfeltétel: ROS 2 Humble, `~/ros2_ws` workspace.

```bash
cd ~/ros2_ws/src
git clone https://github.com/nagyda1/ajr_path_planning
cd ~/ros2_ws
colcon build --packages-select ajr_path_planning --symlink-install
source install/setup.bash
ros2 launch ajr_path_planning planner.launch.py
```

RViz2-ben adjuk hozzá a `/map`, `/astar/markers`, `/rrt/markers` és `/dynamic_obstacle` topicokat a vizualizációhoz.

## Fejlesztési ütemterv (mérföldkövek)

- [ ] 1-2. hét: alapcsomag, `map_server_node`, üres tervező node-ok
- [ ] 3-5. hét: A* implementáció + marker vizualizáció
- [ ] 6-9. hét: RRT/RRT* implementáció, kinematikai korlátok, A* vs. RRT összehasonlítás
- [ ] 10-12. hét: dinamikus akadály, újratervezés, parkolás-szcenárió
- [ ] 13-14. hét: irodalomkutatás dokumentálása, mérési eredmények, README/wiki véglegesítése, demó

Részletes dokumentáció a repo [Wiki](../../wiki) oldalán készül.

## Hivatkozások / felhasznált források

A projekt során felhasznált külső kódrészletek, algoritmusleírások és irodalmi források forrásmegjelöléssel kerülnek beépítésre (lásd a kódban lévő kommenteket és a Wiki irodalomjegyzékét).

## Licenc

A projekt [MIT licenc](LICENSE) alatt érhető el.

## Tantárgy

Győri Széchenyi István Egyetem – Autonóm Járművek és Robotok (AJR) tárgy, nagy féléves beadandó.
