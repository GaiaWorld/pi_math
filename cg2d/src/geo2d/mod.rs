pub use ncollide2d::bounding_volume::AABB as Rectangle;

pub use crate::boolean::boolean::Operation as BooleanOperation;

use crate::boolean::boolean::BooleanOp;
use nalgebra::{Point2, RealField, Scalar};
use num_traits::Float;
use std::slice;
use triangulation::earcut;

/**
 * 多边形：一堆线段组成的区域
 *    + 简单多边形：全连通；每点2条边；顶点数=边数；任意两条边除了顶点外没有额外的交点；
 *       * 凸多边形。
 *       * 凹多边形：可以看成一堆凸多边形组成，需要找到Convex Decomposition算法
 *    + 弱简单多边形：可以带孔(不全连通)；其他条件同简单多边形
 *    + 非简单多边形：边除了顶点外，还可以自相交。
 */

/**
 * 多边形: 可以带孔
 * TODO: 自相交的情况没测试过!
 * hole_indices: 放的是每个孔多边形在vertices中的索引
 * iter
 * 有三个遍历方法：
 *    + for p in string.points(), 可以依次遍历所有的点
 *    + for (pt1, pt2) = string.lines(), 可以依次遍历所有的边，分别是：(p1,p2), (p2,p3), ... (p(n-1),pn), (pn,p1)
 *    + for (pt1, pt2, pt3) = string.triangles(), 可以依次遍历所有的前后缀点，分别是：(p1,p2,p3), (p2,p3,p4), ... (pn, p1,p2)
 */
#[derive(Clone)]
pub struct Polygon<F>
where
    F: Scalar + RealField + Float,
{
    pub vertices: Vec<Point2<F>>,
    pub hole_indices: Vec<usize>, // 每个洞的起点在vertices数组的索引
}

/**
 * 首尾相连的点集: p1, p2, ..., pn
 * 有三个遍历方法：
 *    + for p in string.points(), 可以依次遍历所有的点
 *    + for (pt1, pt2) = string.lines(), 可以依次遍历所有的边，分别是：(p1,p2), (p2,p3), ... (p(n-1),pn), (pn,p1)
 *    + for (pt1, pt2, pt3) = string.triangles(), 可以依次遍历所有的前后缀点，分别是：(p1,p2,p3), (p2,p3,p4), ... (pn, p1,p2)
 */
pub struct LineString<F>
where
    F: Scalar + RealField + Float,
{
    pub vertices: Vec<Point2<F>>, // 点集
}

impl<F> Polygon<F>
where
    F: Scalar + RealField + Float,
{
    pub fn new(pts: &[Point2<F>]) -> Self {
        let mut polygon = Self {
            vertices: vec![],
            hole_indices: vec![],
        };

        polygon.vertices.extend_from_slice(pts);
        polygon
    }

    pub fn new_from_linestring(exterior: &LineString<F>) -> Self {
        let mut polygon = Self {
            vertices: vec![],
            hole_indices: vec![],
        };

        polygon
            .vertices
            .extend_from_slice(exterior.vertices.as_slice());
        polygon
    }

    /**
     * 两个多边形的布尔运算：并，交，差，异或
     */
    pub fn boolean(
        p1: &Polygon<F>,
        p2: &Polygon<F>,
        operation: BooleanOperation,
    ) -> Vec<Polygon<F>> {
        p1.boolean(p2, operation)
    }

    /**
     * 三角剖分
     * 返回polygon里面的顶点的三角形索引列表
     */
    pub fn triangulation(&self) -> Vec<usize> {
        let ptr = self.vertices.as_ptr();
        let ptr = ptr as *const F;
        let pts: &[F] = unsafe { slice::from_raw_parts(ptr, 2 * self.vertices.len()) };
        earcut(pts, &self.hole_indices, 2)
    }

    // 设置外围多边形
    pub fn set_exterior(&mut self, string: &LineString<F>) {
        debug_assert!(self.vertices.len() == 0, "invalid time to set");
        self.vertices.extend_from_slice(string.vertices.as_slice());
    }

    // 插入一个孔
    pub fn push_hole(&mut self, string: &LineString<F>) {
        self.hole_indices.push(self.vertices.len());
        self.vertices.extend_from_slice(string.vertices.as_slice());
    }

    // 外围多边形：得到一个多边形迭代器
    pub fn exterior(&self) -> PolygonIter<F> {
        let hole_num = self.hole_indices.len();
        if hole_num > 0 {
            PolygonIter::<F>::new(&self.vertices[0..self.hole_indices[0]])
        } else {
            PolygonIter::<F>::new(&self.vertices[..])
        }
    }

    // 孔多边形的数量
    pub fn hole_num(&self) -> usize {
        self.hole_indices.len()
    }

    // 第i个孔多边形的迭代器
    pub fn hole(&self, i: usize) -> PolygonIter<F> {
        let len = self.hole_indices.len();
        if i >= len {
            PolygonIter::<F>::new(&self.vertices[0..0])
        } else {
            let start = self.hole_indices[i];
            let end = if i + 1 == len {
                self.vertices.len()
            } else {
                self.hole_indices[i + 1]
            };

            PolygonIter::<F>::new(&self.vertices[start..end])
        }
    }
}

impl<F> Default for Polygon<F>
where
    F: Scalar + RealField + Float,
{
    fn default() -> Self {
        Self {
            vertices: vec![],
            hole_indices: vec![],
        }
    }
}

impl<F> Default for LineString<F>
where
    F: Scalar + RealField + Float,
{
    fn default() -> Self {
        Self { vertices: vec![] }
    }
}

impl<F> LineString<F>
where
    F: Scalar + RealField + Float,
{
    pub fn new(vertices: Vec<Point2<F>>) -> Self {
        Self { vertices }
    }

    pub fn push_point(&mut self, vertex: Point2<F>) {
        self.vertices.push(vertex)
    }

    pub fn points(&self) -> PointIterator<F> {
        PointIterator::<F> {
            cursor: 0,
            vertices: &self.vertices,
        }
    }

    pub fn lines(&self) -> LineIterator<F> {
        LineIterator::<F> {
            cursor: 0,
            vertices: &self.vertices,
        }
    }

    pub fn triangles(&self) -> TriIterator<F> {
        TriIterator::<F> {
            cursor: 0,
            vertices: &self.vertices,
        }
    }
}

pub struct PolygonIter<'a, F: Scalar + RealField + Float> {
    vertices: &'a [Point2<F>],
}

pub struct PointIterator<'a, F: Scalar + RealField + Float> {
    cursor: usize,
    vertices: &'a [Point2<F>],
}

pub struct LineIterator<'a, F: Scalar + RealField + Float> {
    cursor: usize,
    vertices: &'a [Point2<F>],
}

pub struct TriIterator<'a, F: Scalar + RealField + Float> {
    cursor: usize,
    vertices: &'a [Point2<F>],
}

impl<'a, F: Scalar + RealField + Float> PolygonIter<'a, F> {
    pub fn new(vertices: &'a [Point2<F>]) -> Self {
        Self { vertices }
    }

    pub fn points(&self) -> PointIterator<F> {
        PointIterator::<F> {
            cursor: 0,
            vertices: &self.vertices,
        }
    }

    pub fn lines(&self) -> LineIterator<F> {
        LineIterator::<F> {
            cursor: 0,
            vertices: &self.vertices,
        }
    }

    pub fn triangles(&self) -> TriIterator<F> {
        TriIterator::<F> {
            cursor: 0,
            vertices: &self.vertices,
        }
    }
}

impl<'a, F: Scalar + RealField + Float> Iterator for PointIterator<'a, F> {
    type Item = &'a Point2<F>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.cursor < self.vertices.len() {
            self.cursor += 1;
            Some(&self.vertices[self.cursor - 1])
        } else {
            None
        }
    }
}

impl<'a, F: Scalar + RealField + Float> Iterator for LineIterator<'a, F> {
    type Item = (&'a Point2<F>, &'a Point2<F>);

    fn next(&mut self) -> Option<Self::Item> {
        let cursor = self.cursor;
        let len = self.vertices.len();
        if len < 2 || cursor >= len {
            return None;
        }

        self.cursor += 1;
        let pts = self.vertices;
        if 1 + cursor < len {
            Some((&pts[cursor], &pts[cursor + 1]))
        } else if len > 2 {
            Some((&pts[cursor], &pts[0]))
        } else {
            None
        }
    }
}

impl<'a, F: Scalar + RealField + Float> Iterator for TriIterator<'a, F> {
    type Item = (&'a Point2<F>, &'a Point2<F>, &'a Point2<F>);

    fn next(&mut self) -> Option<Self::Item> {
        let cursor = self.cursor;
        let len = self.vertices.len();
        if len < 3 || cursor >= len {
            return None;
        }

        self.cursor += 1;
        let pts = self.vertices;
        if 2 + cursor < len {
            Some((&pts[cursor], &pts[cursor + 1], &pts[cursor + 2]))
        } else if 2 + cursor == len && len > 3 {
            Some((&pts[cursor], &pts[cursor + 1], &pts[0]))
        } else if 1 + cursor == len && len > 3 {
            Some((&pts[cursor], &pts[0], &pts[1]))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::Point2;

    #[test]
    fn test_linestring_iter() {
        let pts = vec![
            Point2::<f32>::new(-2.0, -2.0),
            Point2::<f32>::new(2.0, -2.0),
            Point2::<f32>::new(2.0, 2.0),
            Point2::<f32>::new(-2.0, 2.0),
        ];

        let line_string = LineString::new(pts.clone());

        let mut iter = line_string.points();
        assert_eq!(iter.next(), Some(&pts[0]));
        assert_eq!(iter.next(), Some(&pts[1]));
        assert_eq!(iter.next(), Some(&pts[2]));
        assert_eq!(iter.next(), Some(&pts[3]));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.lines();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1])));
        assert_eq!(iter.next(), Some((&pts[1], &pts[2])));
        assert_eq!(iter.next(), Some((&pts[2], &pts[3])));
        assert_eq!(iter.next(), Some((&pts[3], &pts[0])));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.triangles();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1], &pts[2])));
        assert_eq!(iter.next(), Some((&pts[1], &pts[2], &pts[3])));
        assert_eq!(iter.next(), Some((&pts[2], &pts[3], &pts[0])));
        assert_eq!(iter.next(), Some((&pts[3], &pts[0], &pts[1])));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_linestring_1_pts_iter() {
        let pts = vec![Point2::<f32>::new(-2.0, -2.0)];

        let line_string = LineString::new(pts.clone());

        let mut iter = line_string.points();
        assert_eq!(iter.next(), Some(&pts[0]));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.lines();
        assert_eq!(iter.next(), None);

        let mut iter = line_string.triangles();
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_linestring_2_pts_iter() {
        let pts = vec![
            Point2::<f32>::new(-2.0, -2.0),
            Point2::<f32>::new(2.0, -2.0),
        ];

        let line_string = LineString::new(pts.clone());

        let mut iter = line_string.points();
        assert_eq!(iter.next(), Some(&pts[0]));
        assert_eq!(iter.next(), Some(&pts[1]));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.lines();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1])));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.triangles();
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_linestring_3_pts_iter() {
        let pts = vec![
            Point2::<f32>::new(-2.0, -2.0),
            Point2::<f32>::new(2.0, -2.0),
            Point2::<f32>::new(2.0, 2.0),
        ];

        let line_string = LineString::new(pts.clone());

        let mut iter = line_string.points();
        assert_eq!(iter.next(), Some(&pts[0]));
        assert_eq!(iter.next(), Some(&pts[1]));
        assert_eq!(iter.next(), Some(&pts[2]));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.lines();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1])));
        assert_eq!(iter.next(), Some((&pts[1], &pts[2])));
        assert_eq!(iter.next(), Some((&pts[2], &pts[0])));
        assert_eq!(iter.next(), None);

        let mut iter = line_string.triangles();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1], &pts[2])));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_polygon_iter() {
        let pts = vec![
            Point2::<f32>::new(-2.0, -2.0),
            Point2::<f32>::new(2.0, -2.0),
            Point2::<f32>::new(2.0, 2.0),
            Point2::<f32>::new(-2.0, 2.0),
        ];

        let polygon = Polygon::<f32>::new(&pts);
        assert_eq!(polygon.hole_num(), 0);

        let piter = polygon.exterior();
        let mut iter = piter.points();
        assert_eq!(iter.next(), Some(&pts[0]));
        assert_eq!(iter.next(), Some(&pts[1]));
        assert_eq!(iter.next(), Some(&pts[2]));
        assert_eq!(iter.next(), Some(&pts[3]));
        assert_eq!(iter.next(), None);

        let mut iter = piter.lines();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1])));
        assert_eq!(iter.next(), Some((&pts[1], &pts[2])));
        assert_eq!(iter.next(), Some((&pts[2], &pts[3])));
        assert_eq!(iter.next(), Some((&pts[3], &pts[0])));
        assert_eq!(iter.next(), None);

        let mut iter = piter.triangles();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1], &pts[2])));
        assert_eq!(iter.next(), Some((&pts[1], &pts[2], &pts[3])));
        assert_eq!(iter.next(), Some((&pts[2], &pts[3], &pts[0])));
        assert_eq!(iter.next(), Some((&pts[3], &pts[0], &pts[1])));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_polygon_hole_iter() {
        let pts = vec![
            Point2::<f32>::new(-2.0, -2.0),
            Point2::<f32>::new(2.0, -2.0),
            Point2::<f32>::new(2.0, 2.0),
            Point2::<f32>::new(-2.0, 2.0),
        ];

        let hole1 = vec![
            Point2::<f32>::new(-1.0, -1.0),
            Point2::<f32>::new(-1.0, 1.0),
            Point2::<f32>::new(1.0, 1.0),
            Point2::<f32>::new(1.0, -1.0),
        ];

        let hole2 = vec![
            Point2::<f32>::new(1.7, 1.7),
            Point2::<f32>::new(1.9, 1.2),
            Point2::<f32>::new(1.7, 1.2),
        ];

        let mut polygon = Polygon::<f32>::new(&pts);
        polygon.push_hole(&LineString::<f32>::new(hole1.clone()));
        polygon.push_hole(&LineString::<f32>::new(hole2.clone()));

        let piter = polygon.exterior();
        let mut iter = piter.triangles();
        assert_eq!(iter.next(), Some((&pts[0], &pts[1], &pts[2])));
        assert_eq!(iter.next(), Some((&pts[1], &pts[2], &pts[3])));
        assert_eq!(iter.next(), Some((&pts[2], &pts[3], &pts[0])));
        assert_eq!(iter.next(), Some((&pts[3], &pts[0], &pts[1])));
        assert_eq!(iter.next(), None);

        assert_eq!(polygon.hole_num(), 2);

        let piter = polygon.hole(0);
        let mut iter = piter.lines();
        assert_eq!(iter.next(), Some((&hole1[0], &hole1[1])));
        assert_eq!(iter.next(), Some((&hole1[1], &hole1[2])));
        assert_eq!(iter.next(), Some((&hole1[2], &hole1[3])));
        assert_eq!(iter.next(), Some((&hole1[3], &hole1[0])));
        assert_eq!(iter.next(), None);

        let piter = polygon.hole(1);
        let mut iter = piter.triangles();
        assert_eq!(iter.next(), Some((&hole2[0], &hole2[1], &hole2[2])));
        assert_eq!(iter.next(), None);
    }
}
