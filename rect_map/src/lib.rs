//! 动态分配/释放的矩形表， 动态装箱算法
//! 这个类的目的是将几个矩形打包成一个大地图，同时尽可能地使所有的内容都最佳化。
// 此类通常用于构建光照贴图、sprite贴图或将几个小纹理打包成大纹理。
// 请注意，此类允许释放分配的矩形：即动态维护映射，因此可以根据矩形的生命周期添加/删除矩形。
// 为了更好的利用空间，需要指定对齐大小，推荐按16来对齐。
// 如果需要在分配的矩形周围留有一个空白，请自行维护。
extern crate slab;

use nalgebra::{Point2, Vector2};
use slab::Slab;

/// 矩形表
pub struct RectMap {
    slab: Slab<Node>,
    align: usize, // 对齐大小
}

impl RectMap {
    pub fn new(mut size: Vector2<usize>, align: usize) -> Self {
        align_size(align, &mut size);
        let mut s = Slab::new();
        s.insert(Node::new(size, Point2::new(0, 0), 0, ContentChild::Empty));
        RectMap {
            slab: s,
            align: align,
        }
    }
    // 获取总大小
    pub fn get_size(&self) -> &Vector2<usize> {
        let node = unsafe { self.slab.get_unchecked(1) };
        &node.size
    }
    // 获得对齐大小
    pub fn get_align(&self) -> usize {
        self.align
    }
    // 获取总的矩形数量
    pub fn count(&self) -> usize {
        let node = unsafe { self.slab.get_unchecked(1) };
        match node.content {
            ContentChild::Child(_, _, _, c) => c,
            _ => 0,
        }
    }
    // 添加一个矩形，返回所在的节点id
    pub fn add(&mut self, size: Vector2<usize>) -> usize {
        match find_node(&self.slab, &size, 1) {
            Some((id, area, pos)) => {
                // 计算对齐大小
                let mut asize = size.clone();
                align_size(self.align, &mut asize);
                // 创建右子节点
                let right = if area.x > asize.x {
                    self.slab.insert(Node::new(
                        Vector2::new(area.x - asize.x, asize.y),
                        Point2::new(pos.x + asize.x, pos.y),
                        id,
                        ContentChild::Empty,
                    ))
                } else {
                    0
                };
                // 创建下子节点
                let bottom = if area.y > asize.y {
                    self.slab.insert(Node::new(
                        Vector2::new(area.x, area.y - asize.y),
                        Point2::new(pos.x, pos.y + asize.y),
                        id,
                        ContentChild::Empty,
                    ))
                } else {
                    0
                };
                let (newid, content) = if id > 1 && right == 0 && bottom == 0 {
                    // 放入指定的大小，该节点仅能放入该size。如果是根节点，则必须放到其左子节点
                    (id, ContentChild::Content(size))
                } else {
                    // 放入指定的大小，劈分该节点， 创建左子节点，左子节点仅能放入该size
                    let left =
                        self.slab
                            .insert(Node::new(asize, pos, id, ContentChild::Content(size)));
                    (left, ContentChild::Child(left, right, bottom, 1))
                };
                let node = unsafe { self.slab.get_unchecked_mut(id) };
                node.content = content;
                let p = node.parent;
                incr_count(&mut self.slab, p);
                newid
            }
            _ => 0,
        }
    }
    // 获取指定id矩形的大小和位置
    pub fn get(&self, id: usize) -> Option<(&Vector2<usize>, &Point2<usize>)> {
        match self.slab.get(id) {
            Some(node) => match node.content {
                ContentChild::Content(ref size) => Some((size, &node.pos)),
                _ => None,
            },
            _ => None,
        }
    }
    // 移除指定id的矩形
    pub fn remove(&mut self, id: usize) -> (Vector2<usize>, Point2<usize>) {
        let node = unsafe { self.slab.get_unchecked_mut(id) };
        let size = match node.content {
            ContentChild::Content(size) => size,
            _ => panic!("invalid content"),
        };
        node.content = ContentChild::Empty;
        let pos = node.pos;
        let p = node.parent;
        attempt_defrag(&mut self.slab, p);
        (size, pos)
    }
    // 扩大总大小，生成新的root节点，其包含原根节点和新的右下子节点
    pub fn extends(&mut self, mut size: Vector2<usize>) -> bool {
        align_size(self.align, &mut size);
        let (oldsize, l, r, b, c) = {
            let root = unsafe { self.slab.get_unchecked_mut(1) };
            let w = if root.size.x > size.x {
                root.size.x
            } else {
                size.x
            };
            let h = if root.size.y > size.y {
                root.size.y
            } else {
                size.y
            };
            if w == root.size.x && h == root.size.y {
                return false;
            }
            match root.content {
                ContentChild::Child(l, r, b, c) => (root.size.clone(), l, r, b, c),
                ContentChild::Empty => {
                    root.size = size;
                    return true;
                }
                _ => panic!("invalid content"),
            }
        };
        // 创建右子节点
        let right = if size.x > oldsize.x {
            self.slab.insert(Node::new(
                Vector2::new(size.x - oldsize.x, oldsize.y),
                Point2::new(oldsize.x, 0),
                1,
                ContentChild::Empty,
            ))
        } else {
            0
        };
        // 创建下子节点
        let bottom = if size.y > oldsize.y {
            self.slab.insert(Node::new(
                Vector2::new(size.x, size.y - oldsize.y),
                Point2::new(0, oldsize.y),
                1,
                ContentChild::Empty,
            ))
        } else {
            0
        };
        // 创建左子节点，左子节点就是原根节点
        let left = self.slab.insert(Node::new(
            oldsize,
            Point2::new(0, 0),
            1,
            ContentChild::Child(l, r, b, c),
        ));
        // 修改原根节点的左右下子节点的parent
        let node = unsafe { self.slab.get_unchecked_mut(l) };
        node.parent = left;
        if r > 0 {
            let node = unsafe { self.slab.get_unchecked_mut(r) };
            node.parent = left;
        }
        if b > 0 {
            let node = unsafe { self.slab.get_unchecked_mut(b) };
            node.parent = left;
        }
        // 修改根节点的大小，并包含原根节点和新的右下子节点
        let root = unsafe { self.slab.get_unchecked_mut(1) };
        root.size = size;
        root.content = ContentChild::Child(left, right, bottom, c);
        true
    }
}

#[derive(Debug, Clone)]
struct Node {
    size: Vector2<usize>,  // 大小
    pos: Point2<usize>,    // 位置
    parent: usize,         // 父节点
    content: ContentChild, // 内容或子节点
}

#[derive(Debug, Clone)]
enum ContentChild {
    Empty,                             // 空
    Content(Vector2<usize>),           // 内容大小
    Child(usize, usize, usize, usize), // 左子节点, 右子节点, 下子节点，包含的矩形总数量
}

impl Node {
    #[inline]
    fn new(size: Vector2<usize>, pos: Point2<usize>, parent: usize, content: ContentChild) -> Node {
        Node {
            size: size,
            pos: pos,
            parent: parent,
            content: content,
        }
    }
}

// 对齐大小
pub fn align_size(align: usize, size: &mut Vector2<usize>) {
    let a = size.x % align;
    if a > 0 {
        size.x += align - a;
    }
    let a = size.y % align;
    if a > 0 {
        size.y += align - a;
    }
}

// 寻找能放下指定大小的可用节点
#[inline]
fn find_node(
    slab: &Slab<Node>,
    size: &Vector2<usize>,
    id: usize,
) -> Option<(usize, Vector2<usize>, Point2<usize>)> {
    let node = unsafe { slab.get_unchecked(id) };
    if size.x > node.size.x || size.y > node.size.y {
        return None;
    }
    match node.content {
        ContentChild::Child(left, right, bottom, _) => {
            let r = find_node(slab, size, left);
            if r != None {
                return r;
            }
            if right > 0 {
                let r = find_node(slab, size, right);
                if r != None {
                    return r;
                }
            }
            if bottom > 0 {
                let r = find_node(slab, size, bottom);
                if r != None {
                    return r;
                }
            }
            None
        }
        ContentChild::Empty => Some((id, node.size.clone(), node.pos.clone())),
        _ => None,
    }
}

#[inline]
fn incr_count(slab: &mut Slab<Node>, id: usize) {
    if id == 0 {
        return;
    }
    let node = unsafe { slab.get_unchecked_mut(id) };
    match node.content {
        ContentChild::Child(l, r, b, c) => node.content = ContentChild::Child(l, r, b, c + 1),
        _ => panic!("invalid content"),
    };
    let p = node.parent;
    incr_count(slab, p)
}

// 尝试回收节点
#[inline]
fn attempt_defrag(slab: &mut Slab<Node>, id: usize) {
    if id == 0 {
        return;
    }
    let node = unsafe { slab.get_unchecked_mut(id) };
    let (l, r, b) = match node.content {
        ContentChild::Child(l, r, b, c) => {
            if c > 1 {
                node.content = ContentChild::Child(l, r, b, c - 1);
                (0, 0, 0)
            } else {
                node.content = ContentChild::Empty;
                (l, r, b)
            }
        }
        _ => panic!("invalid content"),
    };
    let p = node.parent;
    if l > 0 {
        slab.remove(l);
        if r > 0 {
            slab.remove(r);
        }
        if b > 0 {
            slab.remove(b);
        }
    }
    attempt_defrag(slab, p)
}

#[test]
fn test11() {
    let mut map = RectMap::new(Vector2::new(512, 512), 16);
    map.add(Vector2::new(50, 50));
    map.add(Vector2::new(20, 20));
    map.add(Vector2::new(200, 200));
    for i in 1..map.slab.len() + 1 {
        println!("test1, id:{}, node: {:?}", i, map.slab.get(i).unwrap());
    }
    map.remove(10);
    for i in 1..map.slab.len() + 1 {
        println!("test2, id:{}, node: {:?}", i, map.slab.get(i));
    }
    map.extends(Vector2::new(600, 600));
    for i in 1..map.slab.len() + 1 {
        println!("test3, id:{}, node: {:?}", i, map.slab.get(i));
    }
}
