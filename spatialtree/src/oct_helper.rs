//! 四叉相关接口
//! 为抽象叉树（四叉或八叉通用一套代码）而抽离的接口struct
use std::mem;

use nalgebra::{Point3, RealField, Scalar, Vector3};
use ncollide3d::bounding_volume::AABB;
use num_traits::{Float, FromPrimitive};
pub struct OctHelper {}

impl OctHelper {
    /// 计算八叉树的深度
    pub fn get_deap<S: Scalar + RealField + FromPrimitive>(
        d: &mut Vector3<S>,
        loose_layer: usize,
        max_loose: &Vector3<S>,
        deep: usize,
        min_loose: &Vector3<S>,
    ) -> usize {
        let two = S::one() + S::one();
        let x = ((max_loose.x / d.x + S::one()) / two)
            .powf(FromPrimitive::from_usize(loose_layer).unwrap());
        let y = ((max_loose.y / d.y + S::one()) / two)
            .powf(FromPrimitive::from_usize(loose_layer).unwrap());
        let z = ((max_loose.z / d.z + S::one()) / two)
            .powf(FromPrimitive::from_usize(loose_layer).unwrap());
        d.x *= x;
        d.y *= y;
        d.z *= z;
        let deep = if loose_layer < deep {
            // 高于该层的节点，松散值都是用最小值， 也可计算其下每层的八叉节点的大小
            // 八叉节点的大小如果小于最小松散值的2倍， 应该停止向下划分， 因为最小松散值占据了八叉节点的大部分
            // 最大层由设置值和该停止划分的层的最小值
            let mut calc_deep = loose_layer;
            let min = min_loose * two;
            while calc_deep < deep && d.x >= min.x && d.y >= min.y && d.z >= min.z {
                *d = (*d + min_loose) / two;
                calc_deep += 1;
            }
            calc_deep
        } else {
            deep
        };
        deep
    }

    #[inline]
     /// 判定指定向量是否小于最小“松散”尺寸
    pub fn smaller_than_min_loose<S: Scalar + RealField>(
        d: &Vector3<S>,
        min_loose: &Vector3<S>,
    ) -> bool {
        if d.x <= min_loose.x && d.y <= min_loose.y && d.z <= min_loose.z {
            return true;
        };
        return false;
    }

    #[inline]
    /// 指定向量以及最大松散尺寸计算对应的层
    pub fn calc_layer<S: Float + Scalar + RealField>(loose: &Vector3<S>, el: &Vector3<S>) -> usize {
        let x = if el.x == S::zero() {
            usize::max_value()
        } else {
            (loose.x / el.x).to_usize().unwrap()
        };
        let y = if el.y == S::zero() {
            usize::max_value()
        } else {
            (loose.y / el.y).to_usize().unwrap()
        };
        let z = if el.z == S::zero() {
            usize::max_value()
        } else {
            (loose.z / el.z).to_usize().unwrap()
        };
        let min = x.min(y).min(z);
        if min == 0 {
            return 0;
        }
        (mem::size_of::<usize>() << 3) - (min.leading_zeros() as usize) - 1
    }

    #[inline]
    /// 判断所在的子节点
    pub fn get_child<S: Scalar + RealField>(point: Point3<S>, aabb: &AABB<S>) -> usize {
        let mut i: usize = 0;
        if aabb.maxs.x > point.x {
            i += 1;
        }
        if aabb.maxs.y > point.y {
            i += 2;
        }
        if aabb.maxs.z > point.z {
            i += 4;
        }
        i
    }

    #[inline]
    ///  判断所在的子节点
    pub fn get_contain_child<S: Scalar + RealField>(
        parent_aabb: &AABB<S>,
        parent_loose: &Vector3<S>,
        aabb: &AABB<S>,
    ) -> usize {
        Self::get_child(
            Self::get_aabb_center_add_half_loose(parent_aabb, parent_loose),
            aabb,
        )
    }

    #[inline]
    pub fn get_aabb_center_add_half_loose<S: Scalar + RealField>(
        aabb: &AABB<S>,
        loose: &Vector3<S>,
    ) -> Point3<S> {
        let two = S::one() + S::one();
        let x = (aabb.mins.x + aabb.maxs.x + loose.x) / two;
        let y = (aabb.mins.y + aabb.maxs.y + loose.y) / two;
        let z = (aabb.mins.z + aabb.maxs.z + loose.z) / two;
        Point3::new(x, y, z)
    }

    #[inline]
    pub fn get_aabb_center_min_half_loose<S: Scalar + RealField>(
        aabb: &AABB<S>,
        loose: &Vector3<S>,
    ) -> Point3<S> {
        let two = S::one() + S::one();
        let x = (aabb.mins.x + aabb.maxs.x - loose.x) / two;
        let y = (aabb.mins.y + aabb.maxs.y - loose.y) / two;
        let z = (aabb.mins.z + aabb.maxs.z - loose.z) / two;
        Point3::new(x, y, z)
    }

    /// 创建ab的子节点集合
    pub fn get_aabbs_childs<S: Scalar + RealField>(
        aabb: &AABB<S>,
        loose: &Vector3<S>,
    ) -> Vec<AABB<S>> {
        let mut result = Vec::new();
        for index in 0..8 {
            result.push(Self::create_child(aabb, loose, index))
        }

        result
    }

    /// 指定创建ab的子节点
    pub fn create_child<S: Scalar + RealField>(
        aabb: &AABB<S>,
        loose: &Vector3<S>,
        index: usize,
    ) -> AABB<S> {
        let p1 = Self::get_aabb_center_min_half_loose(&aabb, &loose);
        let p2 = Self::get_aabb_center_add_half_loose(&aabb, &loose);
        match index {
            0 => AABB::new(aabb.mins, p2),
            1 => AABB::new(
                Point3::new(p1.x, aabb.mins.y, aabb.mins.z),
                Point3::new(aabb.maxs.x, p2.y, p2.z),
            ),
            2 => AABB::new(
                Point3::new(aabb.mins.x, p1.y, aabb.mins.z),
                Point3::new(p2.x, aabb.maxs.y, p2.z),
            ),
            3 => AABB::new(
                Point3::new(p1.x, p1.y, aabb.mins.z),
                Point3::new(aabb.maxs.x, aabb.maxs.y, p2.z),
            ),
            4 => AABB::new(
                Point3::new(aabb.mins.x, aabb.mins.y, p1.z),
                Point3::new(p2.x, p2.y, aabb.maxs.z),
            ),
            5 => AABB::new(
                Point3::new(p1.x, aabb.mins.y, p1.z),
                Point3::new(aabb.maxs.x, p2.y, aabb.maxs.z),
            ),
            6 => AABB::new(
                Point3::new(aabb.mins.x, p1.y, p1.z),
                Point3::new(p2.x, aabb.maxs.y, aabb.maxs.z),
            ),
            _ => AABB::new(p1, aabb.maxs),
        }
    }
}
