//! 高性能的松散叉树
//！采用二进制掩码 表达xyz的大小， child&1 == 0 表示x为小，否则为大。
//！采用Slab，内部用偏移量来分配八叉节点。这样内存连续，八叉树本身可以快速拷贝。

pub mod oct_helper;
pub mod quad_helper;

extern crate core;
extern crate map;
extern crate slab;

// use map::vecmap::VecMap;
// use map::Map;
// use nalgebra::{Point2, RealField, Scalar, Vector2};
// use ncollide2d::bounding_volume::{BoundingVolume, AABB};
// use num_traits::Float;
pub use oct_helper::*;
pub use quad_helper::*;
// use slab::Slab;
// use std::mem;

#[macro_export]
macro_rules! custom_dimension {
    ($PointN:ident{ $($p_field:ident),+ }, $VectorN:ident{ $($v_field:ident),+ }, $AabbN:ident, $Helper:ident ,$d:expr) => {
        // /// branch节点查询函数的范本，aabb是否相交，参数a是查询参数，参数b是branch节点的aabb， 所以最常用的判断是左闭右开
        // /// 应用方为了功能和性能，应该实现自己需要的branch节点的查询函数， 比如点查询， 球查询， 视锥体查询...
        // #[inline]
        // pub fn intersects<S: Scalar + RealField + Float>(a: &$AabbN<S>, b: &$AabbN<S>) -> bool {
        //     a.intersects(b)
        // }

        // /// aabb的查询函数的参数
        // pub struct AbQueryArgs<S: Scalar + RealField + Float, T> {
        //     aabb: $AabbN<S>,
        //     result: Vec<(usize, T)>,
        // }
        // impl<S: Scalar + RealField + Float, T: Clone> AbQueryArgs<S, T> {
        //     pub fn new(aabb: $AabbN<S>) -> AbQueryArgs<S, T> {
        //         AbQueryArgs {
        //             aabb: aabb,
        //             result: Vec::new(),
        //         }
        //     }

        //     pub fn result(&mut self) -> Vec<(usize, T)> {
        //         mem::replace(&mut self.result, Vec::new())
        //     }
        // }

        // /// ab节点的查询函数, 这里只是一个简单范本，使用了branch节点的查询函数intersects
        // /// 应用方为了功能和性能，应该实现自己需要的ab节点的查询函数， 比如点查询， 球查询-包含或相交， 视锥体查询...
        // pub fn ab_query_func<S: Scalar + RealField + Float, T: Clone>(
        //     arg: &mut AbQueryArgs<S, T>,
        //     id: usize,
        //     aabb: &$AabbN<S>,
        //     bind: &T,
        // ) {
        //     if intersects(&arg.aabb, aabb) {
        //         arg.result.push((id, bind.clone()));
        //     }
        // }

        ///
        /// 叉树结构体
        ///
        /// ### 对`N`的约束
        ///
        /// + 浮点数算术运算，可拷贝，可偏序比较；
        /// + 实际使用的时候就是浮点数字类型，比如：f32/f64；
        ///
        pub struct Tree<S: Scalar + RealField + Float, T> {
            branch_slab: Slab<BranchNode<S>>, //所有分支节点（分支节点中包含该层ab节点列表）
            ab_map: VecMap<AbNode<S, T>>,     //所有存储ab碰撞单位的节点
            max_loose: $VectorN<S>,           //最大松散值，第一层的松散大小
            min_loose: $VectorN<S>,           //最小松散值
            adjust: (usize, usize),           //小于min，节点收缩; 大于max，节点分化。默认(4, 7)
            loose_layer: usize,               // 最小松散值所在的深度
            deep: usize,                      // 最大深度, 推荐12-16
            outer: NodeList, // 和根节点不相交的ab节点列表，及节点数量。 相交的放在root的nodes上了。 该AbNode的parent为0
            dirty: (Vec<Vec<usize>>, usize, usize), // 脏的BranchNode节点, 及脏节点数量，及脏节点的起始层
        }

        impl<S: Scalar + RealField + Float, T> Tree<S, T> {
            ///构建树
            ///
            /// 需传入根节点（即全场景）AB碰撞范围；N维实际距离所表示的最大及最小松散参数；叉树收缩及分裂的阈值；叉树的深度限制
            ///
            /// ### 对`N`的约束
            ///
            /// + 浮点数算术运算，可拷贝，可偏序比较；
            /// + 实际使用的时候就是浮点数字类型，比如：f32/f64；
            ///
            pub fn new(
                root: $AabbN<S>,
                max_loose: $VectorN<S>,
                min_loose: $VectorN<S>,
                adjust_min: usize,
                adjust_max: usize,
                deep: usize,
            ) -> Tree<S, T> {
                let adjust_min = if adjust_min == 0 {
                    ADJUST_MIN
                } else {
                    adjust_min
                };
                let adjust_max = if adjust_max == 0 {
                    ADJUST_MAX
                } else {
                    adjust_max
                };
                let adjust_max = if adjust_min > adjust_max {
                    adjust_min
                } else {
                    adjust_max
                };
                let deep = if deep > DEEP_MAX { DEEP_MAX } else { deep };
                let mut branch_slab = Slab::new();
                let mut d = root.extents();
                // 根据最大 最小 松散值 计算出最小松散值所在的最大的层
                let loose_layer = calc_layer(&max_loose, &min_loose);
                let deep = $Helper::get_deap(&mut d, loose_layer, &max_loose, deep, &min_loose);

                branch_slab.insert(BranchNode::new(root, max_loose.clone(), 0, 0, 0));
                return Tree {
                    branch_slab,
                    ab_map: VecMap::default(),
                    max_loose,
                    min_loose,
                    adjust: (adjust_min, adjust_max),
                    loose_layer,
                    deep,
                    outer: NodeList::new(),
                    dirty: (Vec::new(), 0, usize::max_value()),
                };
            }

            /// 获得叉树总的占有内存的字节数
            pub fn mem_size(&self) -> usize {
                let mut r = self.branch_slab.mem_size()
                    + self.ab_map.mem_size()
                    + self.outer.len() * std::mem::size_of::<usize>();
                for v in self.dirty.0.iter() {
                    r += v.capacity() * std::mem::size_of::<usize>();
                }
                r
            }

            /// 获得节点收缩和分化的阈值
            pub fn get_adjust(&self) -> (usize, usize) {
                (self.adjust.0, self.adjust.1)
            }

            /// 获得该aabb对应的层
            pub fn get_layer(&self, aabb: &$AabbN<S>) -> usize {
                let d = aabb.extents();
                if $Helper::smaller_than_min_loose(&d, &self.min_loose) {
                    return self.deep;
                };

                calc_layer(&self.max_loose, &d)
            }

            /// 指定id，在叉树中添加一个aabb单元及其绑定
            pub fn add(&mut self, id: usize, aabb: $AabbN<S>, bind: T) {
                let layer = self.get_layer(&aabb);
                match self.ab_map.insert(id, AbNode::new(aabb, bind, layer)) {
                    Some(_) => panic!("duplicate id: {}", id),
                    _ => (),
                }
                let next = {
                    let node = unsafe { self.ab_map.get_unchecked_mut(id) };
                    let root = unsafe { self.branch_slab.get_unchecked_mut(1) };
                    if root.aabb.contains(&node.aabb) {
                        // root的ab内
                        set_tree_dirty(
                            &mut self.dirty,
                            down(&mut self.branch_slab, self.adjust.1, self.deep, 1, node, id),
                        );
                    } else if root.aabb.intersects(&node.aabb) {
                        // 相交的放在root的nodes上
                        node.parent = 1;
                        node.next = root.nodes.head;
                        root.nodes.push(id);
                    } else {
                        // 和根节点不相交的ab节点, 该AbNode的parent为0
                        node.next = self.outer.head;
                        self.outer.push(id);
                    }
                    node.next
                };

                if next > 0 {
                    let n = unsafe { self.ab_map.get_unchecked_mut(next) };
                    n.prev = id;
                }
            }

            /// 获取指定id的aabb及其绑定
            /// + 该接口返回Option
            pub fn get(&self, id: usize) -> Option<(&$AabbN<S>, &T)> {
                match self.ab_map.get(id) {
                    Some(node) => Some((&node.aabb, &node.bind)),
                    _ => None,
                }
            }

            /// 获取指定id的aabb及其绑定
            pub unsafe fn get_unchecked(&self, id: usize) -> (&$AabbN<S>, &T) {
                let node = self.ab_map.get_unchecked(id);
                (&node.aabb, &node.bind)
            }

            /// 更新指定id的aabb
            pub fn update(&mut self, id: usize, aabb: $AabbN<S>) -> bool {
                let layer = self.get_layer(&aabb);
                let r = match self.ab_map.get_mut(id) {
                    Some(node) => {
                        node.layer = layer;
                        node.aabb = aabb;
                        update(
                            &mut self.branch_slab,
                            &self.adjust,
                            self.deep,
                            &mut self.outer,
                            &mut self.dirty,
                            id,
                            node,
                        )
                    }
                    _ => return false,
                };
                remove_add(self, id, r);
                true
            }

            /// 移动指定id的aabb，性能比update要略好
            pub fn shift(&mut self, id: usize, distance: $VectorN<S>) -> bool {
                let r = match self.ab_map.get_mut(id) {
                    Some(node) => {
                        node.aabb =
                            $AabbN::new(node.aabb.mins + distance, node.aabb.maxs + distance);
                        update(
                            &mut self.branch_slab,
                            &self.adjust,
                            self.deep,
                            &mut self.outer,
                            &mut self.dirty,
                            id,
                            node,
                        )
                    }
                    _ => return false,
                };
                remove_add(self, id, r);
                true
            }

            /// 获取指定id的可写绑定
            pub unsafe fn get_mut(&mut self, id: usize) -> Option<&mut T> {
                match self.ab_map.get_mut(id) {
                    Some(n) => Some(&mut n.bind),
                    _ => None,
                }
            }

            /// 获取指定id的可写绑定
            pub unsafe fn get_unchecked_mut(&mut self, id: usize) -> &mut T {
                let node = self.ab_map.get_unchecked_mut(id);
                &mut node.bind
            }

            /// 更新指定id的绑定
            pub fn update_bind(&mut self, id: usize, bind: T) -> bool {
                match self.ab_map.get_mut(id) {
                    Some(node) => {
                        node.bind = bind;
                        true
                    }
                    _ => false,
                }
            }

            /// 移除指定id的aabb及其绑定
            pub fn remove(&mut self, id: usize) -> Option<($AabbN<S>, T)> {
                let node = match self.ab_map.remove(id) {
                    Some(n) => n,
                    _ => return None,
                };
                if node.parent > 0 {
                    let (p, c) = {
                        let parent = unsafe { self.branch_slab.get_unchecked_mut(node.parent) };
                        if node.parent_child < $d {
                            // 在节点的childs上
                            match parent.childs[node.parent_child] {
                                ChildNode::Ab(ref mut ab) => {
                                    ab.remove(&mut self.ab_map, node.prev, node.next)
                                }
                                _ => panic!("invalid state"),
                            }
                        } else {
                            // 在节点的nodes上
                            parent.nodes.remove(&mut self.ab_map, node.prev, node.next);
                        }
                        (parent.parent, parent.parent_child)
                    };
                    remove_up(&mut self.branch_slab, self.adjust.0, &mut self.dirty, p, c);
                } else {
                    // 表示在outer上
                    self.outer.remove(&mut self.ab_map, node.prev, node.next);
                }
                Some((node.aabb, node.bind))
            }

            /// 整理方法，只有整理方法才会创建或销毁BranchNode
            pub fn collect(&mut self) {
                let mut count = self.dirty.1;
                if count == 0 {
                    return;
                }
                let min_loose = self.min_loose.clone();
                for i in self.dirty.2..self.dirty.0.len() {
                    let vec = unsafe { self.dirty.0.get_unchecked_mut(i) };
                    let c = vec.len();
                    if c == 0 {
                        continue;
                    }
                    for j in 0..c {
                        let branch_id = unsafe { vec.get_unchecked(j) };
                        collect(
                            &mut self.branch_slab,
                            &mut self.ab_map,
                            &self.adjust,
                            self.deep,
                            *branch_id,
                            self.loose_layer,
                            &min_loose,
                        );
                    }
                    vec.clear();
                    if count <= c {
                        break;
                    }
                    count -= c;
                }
                self.dirty.1 = 0;
                self.dirty.2 = usize::max_value();
            }

            /// 查询空间内及相交的ab节点
            pub fn query<A, B>(
                &self,
                branch_arg: &A,
                branch_func: fn(arg: &A, aabb: &$AabbN<S>) -> bool,
                ab_arg: &mut B,
                ab_func: fn(arg: &mut B, id: usize, aabb: &$AabbN<S>, bind: &T),
            ) {
                query(
                    &self.branch_slab,
                    &self.ab_map,
                    1,
                    branch_arg,
                    branch_func,
                    ab_arg,
                    ab_func,
                )
            }

            /// 查询空间外的ab节点
            pub fn query_outer<B>(
                &self,
                arg: &mut B,
                func: fn(arg: &mut B, id: usize, aabb: &$AabbN<S>, bind: &T),
            ) {
                let mut id = self.outer.head;
                while id > 0 {
                    let ab = unsafe { self.ab_map.get_unchecked(id) };
                    func(arg, id, &ab.aabb, &ab.bind);
                    id = ab.next;
                }
            }

            /// 检查碰撞对，不会检查outer的aabb。一般arg包含1个hashset，用(big, little)做键，判断是否已经计算过。
            // pub fn collision<A>(
            //     &self,
            //     id: usize,
            //     _limit_layer: usize,
            //     arg: &mut A,
            //     func: fn(
            //         arg: &mut A,
            //         a_id: usize,
            //         a_aabb: &$AabbN<S>,
            //         a_bind: &T,
            //         b_id: usize,
            //         b_aabb: &$AabbN<S>,
            //         b_bind: &T,
            //     ) -> bool,
            // ) {
            //     let a = match self.ab_map.get(id) {
            //         Some(ab) => ab,
            //         _ => return,
            //     };
            //     // 先判断root.nodes是否有节点，如果有则遍历root的nodes
            //     let node = unsafe { self.branch_slab.get_unchecked(1) };
            //     collision_list(
            //         &self.ab_map,
            //         id,
            //         &a.aabb,
            //         &a.bind,
            //         arg,
            //         func,
            //         node.nodes.head,
            //     );
            //     // 和同列表节点碰撞
            //     collision_list(&self.ab_map, id, &a.aabb, &a.bind, arg, func, a.next);
            //     let mut prev = a.prev;
            //     while prev > 0 {
            //         let b = unsafe { self.ab_map.get_unchecked(prev) };
            //         func(arg, id, &a.aabb, &a.bind, prev, &b.aabb, &b.bind);
            //         prev = b.prev;
            //     }
            //     // 需要计算是否在重叠区，如果在，则需要上溯检查重叠的兄弟节点。不在，其实也需要上溯检查父的匹配节点，但可以提前计算ab节点的最小层
            //     //}
            // }
        }

        //////////////////////////////////////////////////////本地/////////////////////////////////////////////////////////////////

        #[derive(Debug, Clone)]
        struct NodeList {
            head: usize,
            len: usize,
        }
        impl NodeList {
            #[inline]
            pub fn new() -> NodeList {
                NodeList { head: 0, len: 0 }
            }
            #[inline]
            pub fn len(&self) -> usize {
                self.len
            }
            #[inline]
            pub fn push(&mut self, id: usize) {
                self.head = id;
                self.len += 1;
            }
            #[inline]
            pub fn remove<S: Scalar + RealField + Float, T>(
                &mut self,
                map: &mut VecMap<AbNode<S, T>>,
                prev: usize,
                next: usize,
            ) {
                if prev > 0 {
                    let node = unsafe { map.get_unchecked_mut(prev) };
                    node.next = next;
                } else {
                    self.head = next;
                }
                if next > 0 {
                    let node = unsafe { map.get_unchecked_mut(next) };
                    node.prev = prev;
                }
                self.len -= 1;
            }
        }

        const DEEP_MAX: usize = 16;
        const ADJUST_MIN: usize = 4;
        const ADJUST_MAX: usize = 7;

        #[derive(Debug, Clone)]
        struct BranchNode<S: Scalar + RealField + Float> {
            aabb: $AabbN<S>,        // 包围盒
            loose: $VectorN<S>,     // 本层的松散值
            parent: usize,          // 父八叉节点
            parent_child: usize,    // 对应父八叉节点childs的位置
            childs: Vec<ChildNode>, // 子八叉节点
            layer: usize,           // 表示第几层， 根据aabb大小，决定最低为第几层
            nodes: NodeList,        // 匹配本层大小的ab节点列表，及节点数量
            dirty: usize, // 脏标记, 1-128对应节点被修改。添加了节点，并且某个子八叉节点(AbNode)的数量超过阈值，可能分化。删除了节点，并且自己及其下ab节点的数量超过阈值，可能收缩
        }
        impl<S: Scalar + RealField + Float> BranchNode<S> {
            #[inline]
            pub fn new(
                aabb: $AabbN<S>,
                loose: $VectorN<S>,
                parent: usize,
                child: usize,
                layer: usize,
            ) -> BranchNode<S> {
                let mut node = BranchNode {
                    aabb: aabb,
                    loose: loose,
                    parent: parent,
                    parent_child: child,
                    childs: Vec::with_capacity($d),
                    layer: layer,
                    nodes: NodeList::new(),
                    dirty: 0,
                };
                for _ in 0..$d {
                    node.childs.push(ChildNode::Ab(NodeList::new()));
                }
                node
            }
        }
        #[derive(Debug, Clone)]
        enum ChildNode {
            Branch(usize, usize), // 对应的BranchNode, 及其下ab节点的数量
            Ab(NodeList),         // ab节点列表，及节点数量
        }

        #[derive(Debug, Clone)]
        struct AbNode<S: Scalar + RealField + Float, T> {
            aabb: $AabbN<S>,     // 包围盒
            bind: T,             // 绑定
            layer: usize,        // 表示第几层， 根据aabb大小，决定最低为第几层
            parent: usize,       // 父八叉节点
            parent_child: usize, // 父八叉节点所在的子八叉节点， 8表示不在子八叉节点上
            prev: usize,         // 前ab节点
            next: usize,         // 后ab节点
        }
        impl<S: Scalar + RealField + Float, T> AbNode<S, T> {
            pub fn new(aabb: $AabbN<S>, bind: T, layer: usize) -> AbNode<S, T> {
                AbNode {
                    aabb: aabb,
                    bind: bind,
                    layer: layer,
                    parent: 0,
                    parent_child: $d,
                    prev: 0,
                    next: 0,
                }
            }
        }

        // 计算该aabb对应的层
        #[inline]
        fn calc_layer<S: Scalar + RealField + Float>(
            loose: &$VectorN<S>,
            el: &$VectorN<S>,
        ) -> usize {
            $Helper::calc_layer(loose, el)
        }
        // 判断所在的子节点
        #[inline]
        fn get_contain_child<S: Scalar + RealField + Float, T>(
            parent: &BranchNode<S>,
            node: &AbNode<S, T>,
        ) -> usize {
            $Helper::get_contain_child(&parent.aabb, &parent.loose, &node.aabb)
        }
        // 判断所在的子节点
        #[inline]
        fn get_child<S: Scalar + RealField + Float, T>(
            point: $PointN<S>,
            node: &AbNode<S, T>,
        ) -> usize {
            $Helper::get_child(point, &node.aabb)
        }

        // ab节点下降
        fn down<S: Scalar + RealField + Float, T>(
            slab: &mut Slab<BranchNode<S>>,
            adjust: usize,
            deep: usize,
            branch_id: usize,
            node: &mut AbNode<S, T>,
            id: usize,
        ) -> (usize, usize) {
            let parent = unsafe { slab.get_unchecked_mut(branch_id) };
            if parent.layer >= node.layer {
                node.parent = branch_id;
                node.next = parent.nodes.head;
                parent.nodes.push(id);
                return (0, 0);
            }
            let i: usize = get_contain_child(parent, node);
            match parent.childs[i] {
                ChildNode::Branch(branch, ref mut num) => {
                    *num += 1;
                    return down(slab, adjust, deep, branch, node, id);
                }
                ChildNode::Ab(ref mut list) => {
                    node.parent = branch_id;
                    node.parent_child = i;
                    node.next = list.head;
                    list.push(id);
                    if list.len > adjust && parent.layer < deep {
                        return set_dirty(&mut parent.dirty, i, parent.layer, branch_id);
                    }
                    return (0, 0);
                }
            }
        }

        // 更新aabb
        fn update<S: Scalar + RealField + Float, T>(
            slab: &mut Slab<BranchNode<S>>,
            adjust: &(usize, usize),
            deep: usize,
            outer: &mut NodeList,
            dirty: &mut (Vec<Vec<usize>>, usize, usize),
            id: usize,
            node: &mut AbNode<S, T>,
        ) -> Option<(usize, usize, usize, usize, usize)> {
            let old_p = node.parent;
            if old_p > 0 {
                let old_c = node.parent_child;
                let mut parent = unsafe { slab.get_unchecked_mut(old_p) };
                if node.layer > parent.layer {
                    // ab节点能在当前branch节点的容纳范围
                    if parent.aabb.contains(&node.aabb) {
                        // 获得新位置
                        let child = get_contain_child(parent, node);
                        if old_c == child {
                            return None;
                        }
                        if child < $d {
                            let prev = node.prev;
                            let next = node.next;
                            node.prev = 0;
                            // 移动到兄弟节点
                            match parent.childs[child] {
                                ChildNode::Branch(branch, ref mut num) => {
                                    *num += 1;
                                    node.parent_child = $d;
                                    set_tree_dirty(
                                        dirty,
                                        down(slab, adjust.1, deep, branch, node, id),
                                    );
                                    return Some((old_p, old_c, prev, next, node.next));
                                }
                                ChildNode::Ab(ref mut list) => {
                                    node.parent_child = child;
                                    node.next = list.head;
                                    list.push(id);
                                    if list.len > adjust.1 && node.layer < deep {
                                        set_dirty(&mut parent.dirty, child, parent.layer, id);
                                    }
                                    return Some((old_p, old_c, prev, next, node.next));
                                }
                            }
                        }
                    }
                // 需要向上
                } else if node.layer == parent.layer {
                    if parent.aabb.contains(&node.aabb) {
                        if old_c == $d {
                            return None;
                        }
                        let prev = node.prev;
                        let next = node.next;
                        node.prev = 0;
                        // 从child 移到 nodes
                        node.parent_child = $d;
                        node.next = parent.nodes.head;
                        parent.nodes.push(id);
                        return Some((old_p, old_c, prev, next, node.next));
                    }
                // 在当前节点外
                } else {
                    // 比当前节点大
                };
                let prev = node.prev;
                let next = node.next;
                if old_p > 1 {
                    // 向上移动
                    let mut p = parent.parent;
                    let mut c = parent.parent_child;
                    loop {
                        parent = unsafe { slab.get_unchecked_mut(p) };
                        match parent.childs[c] {
                            ChildNode::Branch(_, ref mut num) => {
                                *num -= 1;
                                if *num < adjust.0 {
                                    let d = set_dirty(&mut parent.dirty, c, parent.layer, p);
                                    if d.1 > 0 {
                                        set_tree_dirty(dirty, d);
                                    }
                                }
                            }
                            _ => panic!("invalid state"),
                        }
                        if parent.layer <= node.layer && parent.aabb.contains(&node.aabb) {
                            node.prev = 0;
                            node.parent_child = $d;
                            set_tree_dirty(dirty, down(slab, adjust.1, deep, p, node, id));
                            return Some((old_p, old_c, prev, next, node.next));
                        }
                        p = parent.parent;
                        c = parent.parent_child;
                        if p == 0 {
                            break;
                        }
                    }
                }
                // 判断根节点是否相交
                if parent.aabb.intersects(&node.aabb) {
                    if old_p == 1 && old_c == $d {
                        return None;
                    }
                    // 相交的放在root的nodes上
                    node.parent = 1;
                    node.next = parent.nodes.head;
                    parent.nodes.push(id);
                } else {
                    node.parent = 0;
                    node.next = outer.head;
                    outer.push(id);
                }
                node.prev = 0;
                node.parent_child = $d;
                return Some((old_p, old_c, prev, next, node.next));
            } else {
                // 边界外物体更新
                let root = unsafe { slab.get_unchecked_mut(1) };
                if root.aabb.intersects(&node.aabb) {
                    // 判断是否相交或包含
                    let prev = node.prev;
                    let next = node.next;
                    node.prev = 0;
                    node.parent_child = $d;
                    if root.aabb.contains(&node.aabb) {
                        set_tree_dirty(dirty, down(slab, adjust.1, deep, 1, node, id));
                    } else {
                        // 相交的放在root的nodes上
                        node.parent = 1;
                        node.next = root.nodes.head;
                        root.nodes.push(id);
                    }
                    Some((0, 0, prev, next, node.next))
                } else {
                    // 表示还在outer上
                    None
                }
            }
        }

        /// 从NodeList中移除，并可能添加
        pub fn remove_add<S: Scalar + RealField + Float, T>(
            tree: &mut Tree<S, T>,
            id: usize,
            r: Option<(usize, usize, usize, usize, usize)>,
        ) {
            // 从NodeList中移除
            if let Some((rid, child, prev, next, cur_next)) = r {
                if rid > 0 {
                    let branch = unsafe { tree.branch_slab.get_unchecked_mut(rid) };
                    if child < $d {
                        match branch.childs[child] {
                            ChildNode::Ab(ref mut ab) => ab.remove(&mut tree.ab_map, prev, next),
                            _ => panic!("invalid state"),
                        }
                    } else {
                        branch.nodes.remove(&mut tree.ab_map, prev, next);
                    }
                } else {
                    tree.outer.remove(&mut tree.ab_map, prev, next);
                }
                if cur_next > 0 {
                    let n = unsafe { tree.ab_map.get_unchecked_mut(cur_next) };
                    n.prev = id;
                }
            }
        }

        // 移除时，向上修改数量，并可能设脏
        #[inline]
        fn remove_up<S: Scalar + RealField + Float>(
            slab: &mut Slab<BranchNode<S>>,
            adjust: usize,
            dirty: &mut (Vec<Vec<usize>>, usize, usize),
            parent: usize,
            child: usize,
        ) {
            if parent == 0 {
                return;
            }
            let (p, c) = {
                let node = unsafe { slab.get_unchecked_mut(parent) };
                match node.childs[child] {
                    ChildNode::Branch(_, ref mut num) => {
                        *num -= 1;
                        if *num < adjust {
                            let d = set_dirty(&mut node.dirty, child, node.layer, parent);
                            if d.1 > 0 {
                                set_tree_dirty(dirty, d);
                            }
                        }
                    }
                    _ => panic!("invalid state"),
                }
                (node.parent, node.parent_child)
            };
            remove_up(slab, adjust, dirty, p, c);
        }

        #[inline]
        fn set_dirty(dirty: &mut usize, index: usize, layer: usize, rid: usize) -> (usize, usize) {
            if *dirty == 0 {
                *dirty |= 1 << index;
                return (layer, rid);
            }
            *dirty |= 1 << index;
            return (0, 0);
        }
        // 设置脏标记
        #[inline]
        fn set_tree_dirty(
            dirty: &mut (Vec<Vec<usize>>, usize, usize),
            (layer, rid): (usize, usize),
        ) {
            if rid == 0 {
                return;
            }
            dirty.1 += 1;
            if dirty.2 > layer {
                dirty.2 = layer;
            }
            if dirty.0.len() <= layer {
                for _ in dirty.0.len()..layer + 1 {
                    dirty.0.push(Vec::new())
                }
            }
            let vec = unsafe { dirty.0.get_unchecked_mut(layer) };
            vec.push(rid);
        }

        // 创建指定的子节点
        fn create_child<S: Scalar + RealField + Float>(
            aabb: &$AabbN<S>,
            loose: &$VectorN<S>,
            layer: usize,
            parent_id: usize,
            loose_layer: usize,
            min_loose: &$VectorN<S>,
            child: usize,
        ) -> BranchNode<S> {
            let two = S::one() + S::one();

            let a = $Helper::create_child(aabb, loose, child);
            let loose = if layer < loose_layer {
                loose / two
            } else {
                min_loose.clone()
            };
            return BranchNode::new(a, loose, parent_id, child, layer + 1);
        }

        // 整理方法，只有整理方法才会创建或销毁BranchNode
        fn collect<S: Scalar + RealField + Float, T>(
            branch_slab: &mut Slab<BranchNode<S>>,
            ab_map: &mut VecMap<AbNode<S, T>>,
            adjust: &(usize, usize),
            deep: usize,
            parent_id: usize,
            loose_layer: usize,
            min_loose: &$VectorN<S>,
        ) {
            let (dirty, childs, ab, loose, layer) = {
                let parent = match branch_slab.get_mut(parent_id) {
                    Some(branch) => branch,
                    _ => return,
                };
                let dirty = parent.dirty;
                if parent.dirty == 0 {
                    return;
                }
                parent.dirty = 0;
                (
                    dirty,
                    parent.childs.clone(),
                    parent.aabb.clone(),
                    parent.loose.clone(),
                    parent.layer,
                )
            };
            for i in 0..$d {
                if dirty & (1 << i) != 0 {
                    match childs[i] {
                        ChildNode::Branch(branch, num) if num < adjust.0 => {
                            let mut list = NodeList::new();
                            if num > 0 {
                                shrink(branch_slab, ab_map, parent_id, i, branch, &mut list);
                            }
                            let parent = unsafe { branch_slab.get_unchecked_mut(parent_id) };
                            parent.childs[i] = ChildNode::Ab(list);
                        }
                        ChildNode::Ab(ref list) if list.len > adjust.1 => {
                            let child_id = split(
                                branch_slab,
                                ab_map,
                                adjust,
                                deep,
                                list,
                                &ab,
                                &loose,
                                layer,
                                parent_id,
                                loose_layer,
                                min_loose,
                                i,
                            );
                            let parent = unsafe { branch_slab.get_unchecked_mut(parent_id) };
                            parent.childs[i] = ChildNode::Branch(child_id, list.len);
                        }
                        _ => (),
                    }
                }
            }
        }
        // 收缩BranchNode
        fn shrink<S: Scalar + RealField + Float, T>(
            branch_slab: &mut Slab<BranchNode<S>>,
            ab_map: &mut VecMap<AbNode<S, T>>,
            parent: usize,
            parent_child: usize,
            branch_id: usize,
            result: &mut NodeList,
        ) {
            let node = branch_slab.remove(branch_id);
            if node.nodes.len > 0 {
                shrink_merge(ab_map, parent, parent_child, &node.nodes, result);
            }
            #[macro_use()]
            macro_rules! child_macro {
                ($i:tt) => {
                    match node.childs[$i] {
                        ChildNode::Ab(ref list) if list.len > 0 => {
                            shrink_merge(ab_map, parent, parent_child, &list, result);
                        }
                        ChildNode::Branch(branch, len) if len > 0 => {
                            shrink(branch_slab, ab_map, parent, parent_child, branch, result);
                        }
                        _ => (),
                    }
                };
            }
            for index in 0..$d {
                child_macro!(index);
            }
        }
        // 合并ab列表到结果列表中
        #[inline]
        fn shrink_merge<S: Scalar + RealField + Float, T>(
            ab_map: &mut VecMap<AbNode<S, T>>,
            parent: usize,
            parent_child: usize,
            list: &NodeList,
            result: &mut NodeList,
        ) {
            let old = result.head;
            result.head = list.head;
            result.len += list.len;
            let mut id = list.head;
            loop {
                let ab = unsafe { ab_map.get_unchecked_mut(id) };
                ab.parent = parent;
                ab.parent_child = parent_child;
                if ab.next == 0 {
                    ab.next = old;
                    break;
                }
                id = ab.next;
            }
            if old > 0 {
                let ab = unsafe { ab_map.get_unchecked_mut(old) };
                ab.prev = id;
            }
        }

        // 分裂出BranchNode
        #[inline]
        fn split<S: Scalar + RealField + Float, T>(
            branch_slab: &mut Slab<BranchNode<S>>,
            ab_map: &mut VecMap<AbNode<S, T>>,
            adjust: &(usize, usize),
            deep: usize,
            list: &NodeList,
            parent_ab: &$AabbN<S>,
            parent_loose: &$VectorN<S>,
            parent_layer: usize,
            parent_id: usize,
            loose_layer: usize,
            min_loose: &$VectorN<S>,
            child: usize,
        ) -> usize {
            let branch = create_child(
                parent_ab,
                parent_loose,
                parent_layer,
                parent_id,
                loose_layer,
                min_loose,
                child,
            );
            let branch_id = branch_slab.insert(branch);
            let branch = unsafe { branch_slab.get_unchecked_mut(branch_id) };
            if split_down(ab_map, adjust.1, deep, branch, branch_id, list) > 0 {
                collect(
                    branch_slab,
                    ab_map,
                    adjust,
                    deep,
                    branch_id,
                    loose_layer,
                    min_loose,
                );
            }
            branch_id
        }
        // 将ab节点列表放到分裂出来的八叉节点上
        fn split_down<S: Scalar + RealField + Float, T>(
            map: &mut VecMap<AbNode<S, T>>,
            adjust: usize,
            deep: usize,
            parent: &mut BranchNode<S>,
            parent_id: usize,
            list: &NodeList,
        ) -> usize {
            let point = $Helper::get_aabb_center_add_half_loose(&parent.aabb, &parent.loose);
            let mut id = list.head;
            while id > 0 {
                let node = unsafe { map.get_unchecked_mut(id) };
                let nid = id;
                id = node.next;
                node.prev = 0;
                if parent.layer >= node.layer {
                    node.parent = parent_id;
                    node.parent_child = $d;
                    node.next = parent.nodes.head;
                    parent.nodes.push(nid);
                    continue;
                }
                id = node.next;
                let i = get_child(point, node);
                match parent.childs[i] {
                    ChildNode::Ab(ref mut list) => {
                        node.parent = parent_id;
                        node.parent_child = i;
                        node.next = list.head;
                        list.push(nid);
                        if list.len > adjust && parent.layer < deep {
                            set_dirty(&mut parent.dirty, i, parent.layer, parent_id);
                        }
                        continue;
                    }
                    _ => panic!("invalid state"),
                }
            }
            fix_prev(map, parent.nodes.head);
            for i in 0..$d {
                match parent.childs[i] {
                    ChildNode::Ab(ref list) => fix_prev(map, list.head),
                    _ => (), // panic
                }
            }
            parent.dirty
        }
        // 修复prev
        #[inline]
        fn fix_prev<S: Scalar + RealField + Float, T>(
            map: &mut VecMap<AbNode<S, T>>,
            mut head: usize,
        ) {
            if head == 0 {
                return;
            }
            let node = unsafe { map.get_unchecked(head) };
            let mut next = node.next;
            while next > 0 {
                let node = unsafe { map.get_unchecked_mut(next) };
                node.prev = head;
                head = next;
                next = node.next;
            }
        }

        // 查询空间内及相交的ab节点
        fn query<S: Scalar + RealField + Float, T, A, B>(
            branch_slab: &Slab<BranchNode<S>>,
            ab_map: &VecMap<AbNode<S, T>>,
            branch_id: usize,
            branch_arg: &A,
            branch_func: fn(arg: &A, aabb: &$AabbN<S>) -> bool,
            ab_arg: &mut B,
            ab_func: fn(arg: &mut B, id: usize, aabb: &$AabbN<S>, bind: &T),
        ) {
            let node = unsafe { branch_slab.get_unchecked(branch_id) };
            let mut id = node.nodes.head;
            while id > 0 {
                let ab = unsafe { ab_map.get_unchecked(id) };
                ab_func(ab_arg, id, &ab.aabb, &ab.bind);
                id = ab.next;
            }
            #[macro_use()]
            macro_rules! child_macro {
                ($a:ident, $i:tt) => {
                    match node.childs[$i] {
                        ChildNode::Branch(branch, ref num) if *num > 0 => {
                            if branch_func(branch_arg, &$a) {
                                query(
                                    branch_slab,
                                    ab_map,
                                    branch,
                                    branch_arg,
                                    branch_func,
                                    ab_arg,
                                    ab_func,
                                );
                            }
                        }
                        ChildNode::Ab(ref list) if list.head > 0 => {
                            if branch_func(branch_arg, &$a) {
                                let mut id = list.head;
                                loop {
                                    let ab = unsafe { ab_map.get_unchecked(id) };
                                    ab_func(ab_arg, id, &ab.aabb, &ab.bind);
                                    id = ab.next;
                                    if id == 0 {
                                        break;
                                    }
                                }
                            }
                        }
                        _ => (),
                    }
                };
            }
            let abs = $Helper::get_aabbs_childs(&node.aabb, &node.loose);
            for (idx, ab) in abs.iter().enumerate() {
                child_macro!(ab, idx);
            }
        }

        // 和指定的列表进行碰撞
        fn collision_list<S: Scalar + RealField + Float, T, A>(
            map: &VecMap<AbNode<S, T>>,
            id: usize,
            aabb: &$AabbN<S>,
            bind: &T,
            arg: &mut A,
            func: fn(
                arg: &mut A,
                a_id: usize,
                a_aabb: &$AabbN<S>,
                a_bind: &T,
                b_id: usize,
                b_aabb: &$AabbN<S>,
                b_bind: &T,
            ) -> bool,
            mut head: usize,
        ) {
            while head > 0 {
                let b = unsafe { map.get_unchecked(head) };
                func(arg, id, aabb, bind, head, &b.aabb, &b.bind);
                head = b.next;
            }
        }
    };
}

// 和指定的节点进行碰撞
// fn collision_node<S: Scalar + RealField + Float, T, A>(
//   branch_slab: &Slab<BranchNode<S>>,
//   ab_map: &Slab<AbNode<S, T>>,
//   id: usize,
//   aabb: &$AabbN<S>,
//   bind: &T,
//   arg: &mut A,
//   func: fn(arg: &mut A, a_id: usize, a_aabb: &$AabbN<S>, a_bind: &T, b_id: usize, b_aabb: &$AabbN<S>, b_bind: &T) -> bool,
//   parent: usize,
//   parent_child: usize,
// ) {

// }

// custom_dimension!(Point2 { x, y }, Vector2 { x, y }, AABB, QuadHelper, 4);
// #[test]
// fn test1() {
//     println!("test1-----------------------------------------");
//     let max = Vector2::new(100f32, 100f32);
//     let min = max / 100f32;
//     let mut tree = Tree::new(
//         AABB::new(
//             Point2::new(-1024f32, -1024f32),
//             Point2::new(3072f32, 3072f32),
//         ),
//         max,
//         min,
//         0,
//         0,
//         0,
//     );
//     for i in 0..1 {
//         tree.add(
//             i + 1,
//             AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//             i + 1,
//         );
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("00000, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("00000, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.collect();
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("00000, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     for i in 1..5 {
//         tree.add(
//             i + 1,
//             AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//             i + 3,
//         );
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("00001, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );

//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 700.0), Point2::new(1000.0, 1400.0)),
//     );

//     tree.update(
//         5,
//         AABB::new(Point2::new(0.0, 1400.0), Point2::new(1000.0, 1470.0)),
//     );
//     tree.update(
//         6,
//         AABB::new(Point2::new(0.0, 1470.0), Point2::new(1000.0, 1540.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     tree.collect();
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("00002, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     //   tree.update(1, AABB::new(Point2::new(0.0,0.0,0.0), Point2::new(1000.0, 800.0, 1.0)));
//     //   tree.update(2, AABB::new(Point2::new(0.0,0.0,0.0), Point2::new(1000.0, 800.0, 1.0)));
//     //   tree.update(3, AABB::new(Point2::new(0.0,0.0,0.0), Point2::new(1000.0, 800.0, 1.0)));
//     //   tree.update(4, AABB::new(Point2::new(0.0,0.0,0.0), Point2::new(1000.0, 800.0, 1.0)));

//     //   tree.update(5, AABB::new(Point2::new(0.0,800.0,0.0), Point2::new(1000.0, 1600.0, 1.0)));

//     //    tree.update(6, AABB::new(Point2::new(0.0,1600.0,0.0), Point2::new(1000.0, 2400.0, 1.0)));
//     //   tree.update(7, AABB::new(Point2::new(0.0,2400.0,0.0), Point2::new(1000.0, 3200.0, 1.0)));
//     //   for i in 1..tree.ab_map.len() + 1 {
//     //   println!("22222, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     //  }
//     // tree.collect();
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!(
//             "000000 000000, id:{}, branch: {:?}",
//             i,
//             tree.branch_slab.get(i).unwrap()
//         );
//     }
//     println!("outer:{:?}", tree.outer);
//     let aabb = AABB::new(Point2::new(500f32, 500f32), Point2::new(500f32, 500f32));
//     let mut args: AbQueryArgs<f32, usize> = AbQueryArgs::new(aabb.clone());
//     tree.query(&aabb, intersects, &mut args, ab_query_func);
//     //assert_eq!(args.result(), [1, 3, 4]);
// }

// #[test]
// fn test2() {
//     println!("test2-----------------------------------------");
//     let max = Vector2::new(100f32, 100f32);
//     let min = max / 100f32;
//     let mut tree = Tree::new(
//         AABB::new(
//             Point2::new(-1024f32, -1024f32),
//             Point2::new(3072f32, 3072f32),
//         ),
//         max,
//         min,
//         0,
//         0,
//         0,
//     );
//     for i in 0..9 {
//         tree.add(
//             i + 1,
//             AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//             i + 1,
//         );
//     }
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!("000000, id:{}, branch: {:?}", i, tree.branch_slab.get(i).unwrap());
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("00000, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.update(1, AABB::new(Point2::new(0.0, 0.0), Point2::new(0.0, 0.0)));
//     tree.collect();
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!(
//             "000000 000000, id:{}, branch: {:?}",
//             i,
//             tree.branch_slab.get(i).unwrap()
//         );
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!(
//             "000000 000000, id:{}, ab: {:?}",
//             i,
//             tree.ab_map.get(i).unwrap()
//         );
//     }
//     println!("tree -new ------------------------------------------");
//     let max = Vector2::new(100f32, 100f32);
//     let min = max / 100f32;
//     let mut tree = Tree::new(
//         AABB::new(
//             Point2::new(-1024f32, -1024f32),
//             Point2::new(3072f32, 3072f32),
//         ),
//         max,
//         min,
//         0,
//         0,
//         0,
//     );
//     for i in 0..6 {
//         tree.add(
//             i + 1,
//             AABB::new(Point2::new(0.0, 0.0), Point2::new(0.1, 0.1)),
//             i + 1,
//         );
//     }
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!("test1, id:{}, branch: {:?}", i, tree.branch_slab.get(i).unwrap());
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test1, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.collect();
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!("test2, id:{}, branch: {:?}", i, tree.branch_slab.get(i).unwrap());
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test2, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.shift(4, Vector2::new(2.0, 2.0));
//     tree.shift(5, Vector2::new(4.0, 4.0));
//     tree.shift(6, Vector2::new(10.0, 10.0));
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!("test3, id:{}, branch: {:?}", i, tree.branch_slab.get(i).unwrap());
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test3, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     tree.collect();
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!("test4, id:{}, branch: {:?}", i, tree.branch_slab.get(i).unwrap());
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test4, id:{}, ab: {:?}", i, tree.ab_map.get(i).unwrap());
//     }
//     println!("outer:{:?}", tree.outer);
//     let aabb = AABB::new(Point2::new(0.05f32, 0.05f32), Point2::new(0.05f32, 0.05f32));
//     let mut args: AbQueryArgs<f32, usize> = AbQueryArgs::new(aabb.clone());
//     tree.query(&aabb, intersects, &mut args, ab_query_func);
//     //assert_eq!(args.result(), [1, 2, 3]);
// }

// #[test]
// fn test3() {
//     let aabb = AABB::new(Point2::new(700.0, 100.0), Point2::new(700.0, 100.0));
//     let mut args: AbQueryArgs<f32, usize> = AbQueryArgs::new(aabb.clone());

//     // let mut tree = Tree::new(AABB::new(Point2::new(0f32,0f32,0f32), Point2::new(1000f32,1000f32,1000f32)),
//     // 	0,

//     // 	0,
//     // 	0,
//     // 	0,
//     // );
//     let max = Vector2::new(100f32, 100f32);
//     let min = max / 100f32;
//     let mut tree = Tree::new(
//         AABB::new(Point2::new(0f32, 0f32), Point2::new(1000f32, 1000f32)),
//         max,
//         min,
//         2,
//         4,
//         50,
//     );
//     tree.add(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         2,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.collect();

//     tree.update(
//         0,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(100.0, 700.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(100.0, 700.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(600.0, 600.0), Point2::new(640.0, 640.0)),
//     );

//     tree.add(
//         3,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         4,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         5,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         6,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         7,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         8,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         9,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(40.0, 40.0)),
//         1,
//     );
//     tree.collect();
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!(
//             "test||||||, id:{}, branch: {:?}",
//             i,
//             tree.branch_slab.get(i)
//         );
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test----------, id:{}, ab: {:?}", i, tree.ab_map.get(i));
//     }
//     println!(
//         "-------------------------------------------------------dirtys:{:?}",
//         tree.dirty
//     );
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(100.0, 350.0)),
//     );
//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(80.0, 175.0)),
//     );
//     tree.update(
//         5,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(64.0, 140.0)),
//     );
//     tree.update(
//         6,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(512.0, 112.0)),
//     );
//     tree.update(
//         7,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(410.0, 90.0)),
//     );
//     tree.update(
//         8,
//         AABB::new(Point2::new(800.0, 0.0), Point2::new(160.0, 175.0)),
//     );
//     tree.update(
//         9,
//         AABB::new(Point2::new(800.0, 0.0), Point2::new(144.0, 140.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     tree.query(&aabb, intersects, &mut args, ab_query_func);
//     tree.remove(7);
//     tree.remove(6);
//     tree.remove(5);

//     tree.add(
//         5,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         6,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         7,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.collect();
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!(
//             "test||||||, id:{}, branch: {:?}",
//             i,
//             tree.branch_slab.get(i)
//         );
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test----------, id:{}, ab: {:?}", i, tree.ab_map.get(i));
//     }
//     println!(
//         "-------------------------------------------------------dirtys:{:?}",
//         tree.dirty
//     );
//     tree.update(
//         5,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(640.0, 140.0)),
//     );
//     tree.update(
//         6,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(512.0, 112.0)),
//     );
//     tree.update(
//         7,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(410.0, 90.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(100.0, 700.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(100.0, 700.0)),
//     );
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(100.0, 350.0)),
//     );
//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(80.0, 175.0)),
//     );
//     tree.update(
//         8,
//         AABB::new(Point2::new(800.0, 0.0), Point2::new(160.0, 175.0)),
//     );
//     tree.update(
//         9,
//         AABB::new(Point2::new(800.0, 0.0), Point2::new(140.0, 140.0)),
//     );
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!(
//             "test||||||, id:{}, branch: {:?}",
//             i,
//             tree.branch_slab.get(i)
//         );
//     }
//     for i in 1..tree.ab_map.len() + 1 {
//         println!("test----------, id:{}, ab: {:?}", i, tree.ab_map.get(i));
//     }
//     println!(
//         "-------------------------------------------------------dirtys:{:?}",
//         tree.dirty
//     );
//     tree.query(&aabb, intersects, &mut args, ab_query_func);

//     tree.remove(7);
//     tree.remove(6);
//     tree.remove(5);

//     tree.add(
//         5,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         6,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         7,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)),
//         1,
//     );
//     tree.collect();

//     tree.update(
//         5,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(640.0, 140.0)),
//     );
//     tree.update(
//         6,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(512.0, 112.0)),
//     );
//     tree.update(
//         7,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(410.0, 90.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 700.0)),
//     );
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(1000.0, 350.0)),
//     );
//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 0.0), Point2::new(800.0, 175.0)),
//     );
//     tree.update(
//         8,
//         AABB::new(Point2::new(800.0, 0.0), Point2::new(1600.0, 175.0)),
//     );
//     tree.update(
//         9,
//         AABB::new(Point2::new(800.0, 0.0), Point2::new(1440.0, 140.0)),
//     );
//     tree.query(&aabb, intersects, &mut args, ab_query_func);
//     for i in 1..tree.branch_slab.len() + 1 {
//         println!(
//             "test||||||, id:{}, branch: {:?}",
//             i,
//             tree.branch_slab.get(i)
//         );
//     }
//     for i in 1..tree.ab_map.len() + 10 {
//         println!("test----------, id:{}, ab: {:?}", i, tree.ab_map.get(i));
//     }
//     println!(
//         "-------------------------------------------------------outer:{:?}",
//         tree.outer
//     );
// }

// #[cfg(test)]
// extern crate pcg_rand;
// #[cfg(test)]
// extern crate rand;
// #[test]
// fn test4() {
//     use rand;
//     use rand::Rng;
//     let z_max: f32 = 4194304.0;
//     let aabb = AABB::new(
//         Point2::new(700.0, 100.0, -z_max),
//         Point2::new(700.0, 100.0, z_max),
//     );
//     let mut args: AbQueryArgs<f32, usize> = AbQueryArgs::new(aabb.clone());

//     let max = Vector2::new(100f32, 100f32, 100f32);
//     let min = max / 100f32;
//     let mut tree = Tree::new(
//         AABB::new(
//             Point2::new(-1024f32, -1024f32, -4194304f32),
//             Point2::new(3072f32, 3072f32, 4194304f32),
//         ),
//         max,
//         min,
//         0,
//         0,
//         0,
//     );
//     tree.add(
//         1,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         2,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.collect();

//     tree.update(
//         0,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );

//     tree.add(
//         3,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         4,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         5,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         6,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         7,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         8,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         9,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.collect();
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 350.0, 1.0)),
//     );
//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(800.0, 175.0, 1.0)),
//     );
//     tree.update(
//         5,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(640.0, 140.0, 1.0)),
//     );
//     tree.update(
//         6,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(512.0, 112.0, 1.0)),
//     );
//     tree.update(
//         7,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(410.0, 90.0, 1.0)),
//     );
//     tree.update(
//         8,
//         AABB::new(
//             Point2::new(800.0, 0.0, 0.0),
//             Point2::new(1600.0, 175.0, 1.0),
//         ),
//     );
//     tree.update(
//         9,
//         AABB::new(
//             Point2::new(800.0, 0.0, 0.0),
//             Point2::new(1440.0, 140.0, 1.0),
//         ),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );

//     tree.add(
//         10,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         11,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         12,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.collect();
//     tree.update(
//         10,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(640.0, 140.0, 1.0)),
//     );
//     tree.update(
//         11,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(512.0, 112.0, 1.0)),
//     );
//     tree.update(
//         12,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(410.0, 90.0, 1.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 350.0, 1.0)),
//     );
//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(800.0, 175.0, 1.0)),
//     );
//     tree.update(
//         8,
//         AABB::new(
//             Point2::new(800.0, 0.0, 0.0),
//             Point2::new(1600.0, 175.0, 1.0),
//         ),
//     );
//     tree.update(
//         9,
//         AABB::new(
//             Point2::new(800.0, 0.0, 0.0),
//             Point2::new(1440.0, 140.0, 1.0),
//         ),
//     );

//     tree.add(
//         13,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         14,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.add(
//         15,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1.0, 1.0, 1.0)),
//         1,
//     );
//     tree.collect();

//     log(&tree.branch_slab, &tree.ab_map, 10000);
//     tree.update(
//         13,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(640.0, 140.0, 1.0)),
//     );
//     log(&tree.branch_slab, &tree.ab_map, 13);
//     println!("branch========================");
//     tree.update(
//         14,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(512.0, 112.0, 1.0)),
//     );
//     tree.update(
//         15,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(410.0, 90.0, 1.0)),
//     );
//     tree.update(
//         1,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         2,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 700.0, 1.0)),
//     );
//     tree.update(
//         3,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(1000.0, 350.0, 1.0)),
//     );
//     tree.update(
//         4,
//         AABB::new(Point2::new(0.0, 0.0, 0.0), Point2::new(800.0, 175.0, 1.0)),
//     );
//     tree.update(
//         8,
//         AABB::new(
//             Point2::new(800.0, 0.0, 0.0),
//             Point2::new(1600.0, 175.0, 1.0),
//         ),
//     );
//     tree.update(
//         9,
//         AABB::new(
//             Point2::new(800.0, 0.0, 0.0),
//             Point2::new(1440.0, 140.0, 1.0),
//         ),
//     );
//     let mut rng = rand::thread_rng();
//     for _ in 0..1000000 {
//         let r = rand::thread_rng().gen_range(1, 16);
//         let x = rand::thread_rng().gen_range(0, 128) - 64;
//         let y = rand::thread_rng().gen_range(0, 128) - 64;
//         let old = clone_tree(&tree.branch_slab, &tree.ab_map);
//         tree.shift(r, Vector2::new(x as f32, y as f32, 0.0));
//         assert_eq!(check_tree(&tree.branch_slab, &tree.ab_map, old, r), false);
//         if x + y > 120 {
//             let old = clone_tree(&tree.branch_slab, &tree.ab_map);
//             tree.collect();
//             assert_eq!(check_tree(&tree.branch_slab, &tree.ab_map, old, r), false);
//         }
//     }
// }
// #[cfg(test)]
// fn clone_tree(
//     branch_slab: &Slab<BranchNode<f32>>,
//     ab_map: &VecMap<AbNode<f32, usize>>,
// ) -> (Slab<BranchNode<f32>>, VecMap<AbNode<f32, usize>>) {
//     (branch_slab.clone(), ab_map.clone())
// }

// #[cfg(test)]
// fn check_tree(
//     branch_slab: &Slab<BranchNode<f32>>,
//     ab_map: &VecMap<AbNode<f32, usize>>,
//     old: (Slab<BranchNode<f32>>, VecMap<AbNode<f32, usize>>),
//     r: usize,
// ) -> bool {
//     for (id, _n) in branch_slab.iter() {
//         if check_ab(branch_slab, ab_map, id) {
//             log(&old.0, &old.1, r);
//             log(branch_slab, ab_map, id);
//             return true;
//         }
//     }
//     false
// }
// #[cfg(test)]
// fn check_ab(
//     branch_slab: &Slab<BranchNode<f32>>,
//     ab_map: &VecMap<AbNode<f32, usize>>,
//     branch_id: usize,
// ) -> bool {
//     let node = unsafe { branch_slab.get_unchecked(branch_id) };
//     let mut old = VecMap::default();
//     for c in 0..8 {
//         match &node.childs[c] {
//             ChildNode::Ab(list) => {
//                 if check_list(ab_map, branch_id, c, &list, &mut old) {
//                     return true;
//                 };
//             }
//             _ => (),
//         }
//     }
//     check_list(ab_map, branch_id, 8, &node.nodes, &mut old)
// }
// #[cfg(test)]
// fn check_list(
//     ab_map: &VecMap<AbNode<f32, usize>>,
//     parent: usize,
//     parent_child: usize,
//     list: &NodeList,
//     old: &mut VecMap<usize>,
// ) -> bool {
//     let mut id = list.head;
//     let mut prev = 0;
//     let mut i = 0;
//     while id > 0 {
//         old.insert(id, id);
//         let ab = unsafe { ab_map.get_unchecked(id) };
//         if ab.prev != prev {
//             println!("------------0-branch_id: {}, ab_id: {}", parent, id);
//             return true;
//         }
//         if ab.parent != parent {
//             println!("------------1-branch_id: {}, ab_id: {}", parent, id);
//             return true;
//         }
//         if ab.parent_child != parent_child {
//             println!("------------2-branch_id: {}, ab_id: {}", parent, id);
//             return true;
//         }
//         if old.contains(ab.next) {
//             println!("------------3-branch_id: {}, ab_id: {}", parent, id);
//             return true;
//         }
//         prev = id;
//         id = ab.next;
//         i += 1;
//     }
//     if i != list.len {
//         println!("------------4-branch_id: {}, ab_id: {}", parent, id);
//         return true;
//     }
//     return false;
// }

// #[cfg(test)]
// fn log(branch_slab: &Slab<BranchNode<f32>>, ab_map: &VecMap<AbNode<f32, usize>>, branch_id: usize) {
//     println!("branch_id----------, id:{}", branch_id);
//     let mut i = 0;
//     for or in ab_map.iter() {
//         i += 1;
//         if let Some(r) = or {
//             //let r = ab_map.get(r).unwrap();
//             println!("ab----------, id:{}, ab: {:?}", i, r);
//         }
//     }
//     for (id, n) in branch_slab.iter() {
//         //let r = ab_map.get(r).unwrap();
//         println!("branch=========, id:{}, branch: {:?}", id, n);
//     }
// }

// #[test]
// fn test_update() {
//     use pcg_rand::Pcg32;
//     use rand::{Rng, SeedableRng};

//     let max_size = 1000.0;

//     let max = Vector2::new(100f32, 100f32, 100f32);
//     let min = max / 100f32;
//     let mut tree = Tree::new(
//         AABB::new(
//             Point2::new(0.0, 0.0, 0.0),
//             Point2::new(max_size, max_size, max_size),
//         ),
//         max,
//         min,
//         0,
//         0,
//         10,
//     );

//     let mut rng = pcg_rand::Pcg32::seed_from_u64(1111);
//     //println!("rr = {}", rr);
//     for i in 0..10000 {
//         //println!("i = {}", i);

//         let x = rng.gen_range(0.0, max_size);
//         let y = rng.gen_range(0.0, max_size);
//         let z = rng.gen_range(0.0, max_size);

//         tree.add(
//             i + 1,
//             AABB::new(Point2::new(x, y, z), Point2::new(x, y, z)),
//             i + 1,
//         );

//         tree.collect();

//         let x_: f32 = rng.gen_range(0.0, max_size);
//         let y_: f32 = rng.gen_range(0.0, max_size);
//         let z_: f32 = rng.gen_range(0.0, max_size);

//         // TODO: 改成 7.0 就可以了。
//         let size: f32 = 1.0;
//         let aabb = AABB::new(
//             Point2::new(x_, y_, z_),
//             Point2::new(x_ + size, y_ + size, z_ + size),
//         );

//         tree.update(i + 1, aabb.clone());
//         //tree.remove(i + 1);
//         //tree.add(i + 1, aabb.clone(), i + 1);
//         // if i == 25 {
//         //     let old = clone_tree(&tree.branch_slab, &tree.ab_map);
//         //     assert_eq!(check_tree(&tree.branch_slab, &tree.ab_map, old, i), false);
//         //     log(&tree.branch_slab, &tree.ab_map, i);
//         // }
//         tree.collect();
//         // let aabb = AABB::new(
//         //     Point2::new(aabb.min.x - 1.0, aabb.min.y - 1.0, aabb.min.z - 1.0),
//         //     Point2::new(aabb.min.x + 1.0, aabb.min.y + 1.0, aabb.min.z + 1.0),
//         // );
//         // if i == 25 {
//         //     let old = clone_tree(&tree.branch_slab, &tree.ab_map);
//         //     assert_eq!(check_tree(&tree.branch_slab, &tree.ab_map, old, i), false);
//         //     log(&tree.branch_slab, &tree.ab_map,i);
//         // }
//         let mut args: AbQueryArgs<f32, usize> = AbQueryArgs::new(aabb.clone());
//         tree.query(&aabb, intersects, &mut args, ab_query_func);
//         assert!(args.result().len() > 0);
//     }
// }
