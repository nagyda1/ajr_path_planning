# A* és RRT mérési kiértékelés

A kiértékelés a `planning_results_12cel.csv` fájl mérési adataiból készült.

| Mutató | A* | RRT |
|---|---:|---:|
| Mérések száma | 12 | 12 |
| Sikerességi arány | 100.0% | 100.0% |
| Útvonalhossz (m) | 8.412 | 11.375 |
| Futásidő (ms) | 1.225 | 5.885 |
| Útvonalpontok száma | 16.167 | 24.417 |
| Fa csomópontok száma | 0.000 | 116.167 |

A futásidő ezredmásodpercben, az útvonalhossz méterben értendő. Az A* fa-csomópontszáma definíció szerint 0; az RRT értéke a generált keresőfa méretét jelzi.
