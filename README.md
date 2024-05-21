# QClique: Optimizing Performance and Accuracy in Maximum Weighted Clique
 
The Maximum Weighted Clique (MWC) problem remains challenging due to its unfavourable time complexity. In this paper, we analyze the execution of exact search-based MWC algorithms and show that high-accuracy weighted cliques can be discovered in the early stages of the execution if searching the combinatorial space is performed systematically. Based on this observation, we introduce QClique as an approximate MWC algorithm that processes the search space as long as better cliques are expected. QClique uses a tunable parameter to trade-off between accuracy vs. execution time and delivers 4.7−82.3× speedup in comparison to previous state-of-the-art MWC algorithms while providing 91.4% accuracy and achieves a parallel speedup of up to 56× on 128 threads. Additionally, QClique accelerates the exact MWC computation by replacing the initial clique of the exact algorithm. For WLMC, an exact state-of-the-art MWC algorithm, this results in 3.3× on average.
 
 
### How to run:
```
g++ -std=c++11 QClique.cpp -fopenmp -o QClique
./QClique  -p 128
```
 
 
### Bibtex:
```
@inproceedings{QClique,
  title = { {QClique}: Optimizing Performance and Accuracy in Maximum Weighted Clique}, 
  author = { Qasim Abbas and {Mohsen} {Koohi Esfahani}  and Ian Overton and Hans Vandierendonck},
  year = {2024},
  booktitle = {Euro-Par 2024},
}
```

### License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE.txt) file for details.
