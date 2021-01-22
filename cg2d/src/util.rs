use nalgebra::{Point2, RealField, Scalar, Vector2};
use num_traits::Float;

pub enum InnOuter {
    Inner,
    Outer,
    Boundary,
}

/// 检查点是否在三角形范围内， 要求p1 p2 p3 要么顺时针要么逆时针
pub fn include_tri2<S>(p: &Point2<S>, p1: &Point2<S>, p2: &Point2<S>, p3: &Point2<S>) -> InnOuter
where
    S: Float + Scalar + RealField,
{
    let zero = S::zero();
    let v1 = Vector2::new(p.x - p1.x, p.y - p1.y);
    let v2 = Vector2::new(p.x - p2.x, p.y - p2.y);
    let v3 = Vector2::new(p.x - p3.x, p.y - p3.y);
    // 要求3次叉乘结果，要么都大于0 要么都小于0
    let r = v1.cross(&v2);
    if r.max() > zero {
        let r = v2.cross(&v3);
        if r.max() > zero {
        } else if r.max() < zero {
            return InnOuter::Outer;
        } else {
            return InnOuter::Boundary;
        }
        let r = v3.cross(&v1);
        if r.max() > zero {
            InnOuter::Inner
        } else if r.max() < zero {
            InnOuter::Outer
        } else {
            InnOuter::Boundary
        }
    } else if r.max() < zero {
        let r = v2.cross(&v3);
        if r.max() < zero {
        } else if r.max() > zero {
            return InnOuter::Outer;
        } else {
            return InnOuter::Boundary;
        }
        let r = v3.cross(&v1);
        if r.max() < zero {
            InnOuter::Inner
        } else if r.max() > zero {
            InnOuter::Outer
        } else {
            InnOuter::Boundary
        }
    } else {
        InnOuter::Boundary
    }
}
/// 检查点是否在四边形范围内， 要求p1 p2 p3 p4 要么顺时针要么逆时针
pub fn include_quad2<S>(
    p: &Point2<S>,
    p1: &Point2<S>,
    p2: &Point2<S>,
    p3: &Point2<S>,
    p4: &Point2<S>,
) -> InnOuter
where
    S: Float + Scalar + RealField,
{
    let zero = S::zero();
    let v1 = Vector2::new(p.x - p1.x, p.y - p1.y);
    let v2 = Vector2::new(p.x - p2.x, p.y - p2.y);
    let v3 = Vector2::new(p.x - p3.x, p.y - p3.y);
    let v4 = Vector2::new(p.x - p4.x, p.y - p4.y);
    // 要求4次叉乘结果，要么都大于0 要么都小于0
    let r = v1.cross(&v2);
    if r.max() > zero {
        let r = v2.cross(&v3);
        if r.max() > zero {
        } else if r.max() < zero {
            return InnOuter::Outer;
        } else {
            return InnOuter::Boundary;
        }
        let r = v3.cross(&v4);
        if r.max() > zero {
        } else if r.max() < zero {
            return InnOuter::Outer;
        } else {
            return InnOuter::Boundary;
        }
        let r = v4.cross(&v1);
        if r.max() > zero {
            InnOuter::Inner
        } else if r.max() < zero {
            InnOuter::Outer
        } else {
            InnOuter::Boundary
        }
    } else if r.max() < zero {
        let r = v2.cross(&v3);
        if r.max() < zero {
        } else if r.max() > zero {
            return InnOuter::Outer;
        } else {
            return InnOuter::Boundary;
        }
        let r = v3.cross(&v4);
        if r.max() < zero {
        } else if r.max() > zero {
            return InnOuter::Outer;
        } else {
            return InnOuter::Boundary;
        }
        let r = v4.cross(&v1);
        if r.max() < zero {
            InnOuter::Inner
        } else if r.max() > zero {
            InnOuter::Outer
        } else {
            InnOuter::Boundary
        }
    } else {
        InnOuter::Boundary
    }
}
