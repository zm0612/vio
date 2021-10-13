# 实验报告

## 1. 完成单目BA求解器中的代码：Problem::MakeHessian()和Problem::SolveLinearSystem()

### 1.1 经验总结

**代码流程**

首先代码(lecture_5_backend_solver/code/app/TestMonoBA.cpp)的整体逻辑比较清晰，大致流程如下：

- 生成仿真数据：路标点和相机姿态；
- 构建顶点：相机姿态、路标点逆深度；
- 构建边：三元边(两个相机姿态+逆深度)的重投影误差；
- 然后构建优化器进行迭代求解。

**核心的难点**

a. 对于边的雅克比计算，PPT中都已经给出了详细的计算过程，可以参考；

b. 对于Hessian矩阵的构建并不是很难，需要格外注意矩阵的索引，只要索引都正确一般没什么难度；

c. 对于Hessian的求解，可以先利用舒尔补求解Pose对应的增量，然后再计算landmark对应的增量，应为landmark是一个稀疏矩阵。对于landmark对应的Hessian矩阵，可以使用分块求逆；

> 以上过程PPT中给了详细的过程，参考PPT比较容易写出来。

### 1.2 实验结果

**a. 不固定任何顶点**

>ordered_landmark_vertices_ size : 20
>iter: 0 , chi= 5.35099 , Lambda= 0.00597396
>iter: 1 , chi= 0.0301337 , Lambda= 0.00199132
>iter: 2 , chi= 0.000848882 , Lambda= 0.00132755
>iter: 3 , chi= 0.000840597 , Lambda= 0.00177006
>iter: 4 , chi= 0.000836098 , Lambda= 0.00236008
>problem solve cost: 0.73899 ms
>makeHessian cost: 0.34461 ms
>
>Compare MonoBA results after opt...
>after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.220367
>after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.234588
>after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt 0.142355
>after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.213878
>after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.130118
>after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt 0.191559
>after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.166945
>after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.201705
>after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt 0.16754
>after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt 0.218986
>after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt 0.205696
>after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt 0.127512
>after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt 0.167932
>after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt 0.216217
>after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.179541
>after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.227175
>after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.157117
>after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt 0.181943
>after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.155248
>after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.146331
>------------ pose translation ----------------
>translation after opt: 0 :-0.0126673 -0.0208772  0.0131711 || gt: 0 0 0
>translation after opt: 1 :-1.05976  4.00208 0.864682 || gt:  -1.0718        4 0.866025
>translation after opt: 2 :-3.98528  6.94819 0.861153 || gt:       -4   6.9282 0.866025

执行结果可以发现，优化之后第一个相机的位置发生了轻微的平移。这确实印证了单目BA问题存在零空间。

**b. 固定两个相机位姿顶点**

> 0 order: 0
> 1 order: 6
> 2 order: 12
>
>  ordered_landmark_vertices_ size : 20
> iter: 0 , chi= 5.35099 , Lambda= 0.00597396
> iter: 1 , chi= 0.0321522 , Lambda= 0.00199132
> iter: 2 , chi= 0.00278558 , Lambda= 0.000663774
> iter: 3 , chi= 0.00277605 , Lambda= 0.000442516
> iter: 4 , chi= 0.00277121 , Lambda= 0.000295011
> problem solve cost: 0.36187 ms
>    makeHessian cost: 0.177168 ms
>
> Compare MonoBA results after opt...
> after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.220595
> after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.234412
> after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt 0.142267
> after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.214268
> after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.130345
> after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt 0.191473
> after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.166913
> after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.201665
> after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt 0.167803
> after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt 0.218787
> after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt 0.205678
> after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt 0.127592
> after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt 0.167877
> after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt 0.216592
> after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.179797
> after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.227095
> after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.15736
> after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt 0.182083
> after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.155373
> after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.146334
> ------------ pose translation ----------------
> translation after opt: 0 :0 0 0 || gt: 0 0 0
> translation after opt: 1 : -1.0718        4 0.866025 || gt:  -1.0718        4 0.866025
> translation after opt: 2 :-3.98127  6.94912 0.856755 || gt:       -4   6.9282 0.866025

固定两个位姿顶点之后，前两个位姿就不发生变化了，固定住了位姿。

**c. 固定一个相机位姿和一个逆深度**













