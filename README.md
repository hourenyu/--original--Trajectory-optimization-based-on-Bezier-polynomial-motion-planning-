# Trajectory-optimization-based-on-Bezier-polynomial-motion-planning-
基于贝赛尔曲线的轨迹优化： 前端：Astar寻径   后端：运动走廊膨胀＋Bezier

1)前端：Astar寻径
  前端path finding使用的是经典的Astar算法，唯一不同的是在搜索的时候对每个node新增了一个与障碍物距离的代价指标，它与启发式代价和累计代价一起决定了node在openlist中的排序。
  
2)后端：运动走廊膨胀＋Bezier优化

  ①为了得到比较稀疏的运动走廊，膨胀分为两步。第一是基于前端得到的path节点进行膨胀，但是加了一个限制条件，即已经被bounding box覆盖的节点不予膨胀，这样的好处是不用再扩展冗余的box以及为了下一步的简化减少了很大的计算量。第二是对得到的bounding box进一步简化，因为前一步得到的bounding box仍然存在大量重叠的情况，对接下来的轨迹优化来说无疑是无用且消耗计算的，这里简化的核心思想是从当前boxNow开始按照顺序找到其不相交的boxA，并将该boxA的前一个boxB保留下来，并删除从boxNow到boxB之间的所有box,然后boxA作为新的boxNow继续删减。
  

  ②Bezier优化主要参考文章为Online Safe Trajectory Generation For Quadrotors　Using Fast Marching Method and Bernstein Basis Polynomial

代码在持续优化中...欢迎大家一起交流。
