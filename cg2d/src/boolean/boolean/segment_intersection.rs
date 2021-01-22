use nalgebra::{Point2, RealField, Scalar};

#[derive(Debug, Clone, PartialEq)]
pub enum LineIntersection<F>
where
    F: Scalar + RealField,
{
    None,
    Point(Point2<F>),
    Overlap(Point2<F>, Point2<F>),
}

pub fn intersection<F>(
    a1: Point2<F>,
    a2: Point2<F>,
    b1: Point2<F>,
    b2: Point2<F>,
) -> LineIntersection<F>
where
    F: Scalar + RealField,
{
    let va = Point2::new(a2.x - a1.x, a2.y - a1.y);
    let vb = Point2::new(b2.x - b1.x, b2.y - b1.y);
    let e = Point2::new(b1.x - a1.x, b1.y - a1.y);
    let mut kross = cross_product(va, vb);
    let mut sqr_kross = kross * kross;
    let sqr_len_a = dot_product(va, va);

    if sqr_kross > F::zero() {
        let s = cross_product(e, vb) / kross;
        if s < F::zero() || s > F::one() {
            return LineIntersection::None;
        }
        let t = cross_product(e, va) / kross;
        if t < F::zero() || t > F::one() {
            return LineIntersection::None;
        }

        if s == F::zero() || s == F::one() {
            return LineIntersection::Point(mid_point(a1, s, va));
        }
        if t == F::zero() || t == F::one() {
            return LineIntersection::Point(mid_point(b1, t, vb));
        }

        return LineIntersection::Point(mid_point(a1, s, va));
    }

    kross = cross_product(e, va);
    sqr_kross = kross * kross;

    if sqr_kross > F::zero() {
        return LineIntersection::None;
    }

    let sa = dot_product(va, e) / sqr_len_a;
    let sb = sa + dot_product(va, vb) / sqr_len_a;
    let smin = sa.min(sb);
    let smax = sa.max(sb);

    if smin <= F::one() && smax >= F::zero() {
        if smin == F::one() {
            return LineIntersection::Point(mid_point(a1, smin, va));
        }
        if smax == F::zero() {
            return LineIntersection::Point(mid_point(a1, smax, va));
        }

        return LineIntersection::Overlap(
            mid_point(a1, smin.max(F::zero()), va),
            mid_point(a1, smax.min(F::one()), va),
        );
    }

    LineIntersection::None
}

fn mid_point<F>(p: Point2<F>, s: F, d: Point2<F>) -> Point2<F>
where
    F: Scalar + RealField,
{
    Point2::new(p.x + s * d.x, p.y + s * d.y)
}

#[inline]
fn cross_product<F>(a: Point2<F>, b: Point2<F>) -> F
where
    F: Scalar + RealField,
{
    a.x * b.y - a.y * b.x
}

#[inline]
fn dot_product<F>(a: Point2<F>, b: Point2<F>) -> F
where
    F: Scalar + RealField,
{
    a.x * b.x + a.y * b.y
}

#[cfg(test)]
mod test {
    use super::super::helper::test::xy;
    use super::*;

    #[test]
    fn test_intersection() {
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 0), xy(2, 2)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 0), xy(10, 2)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(2, 2), xy(3, 3), xy(0, 6), xy(2, 4)),
            LineIntersection::None
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 0), xy(0, 1)),
            LineIntersection::Point(xy(0.5, 0.5))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, 1), xy(0, 0)),
            LineIntersection::Point(xy(0, 0))
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, 1), xy(1, 1)),
            LineIntersection::Point(xy(1, 1))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0.5, 0.5), xy(1, 0)),
            LineIntersection::Point(xy(0.5, 0.5))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(10, 10), xy(1, 1), xy(5, 5)),
            LineIntersection::Overlap(xy(1, 1), xy(5, 5))
        );
        assert_eq!(
            intersection(xy(1, 1), xy(10, 10), xy(1, 1), xy(5, 5)),
            LineIntersection::Overlap(xy(1, 1), xy(5, 5))
        );
        assert_eq!(
            intersection(xy(3, 3), xy(10, 10), xy(0, 0), xy(5, 5)),
            LineIntersection::Overlap(xy(3, 3), xy(5, 5))
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, 0), xy(1, 1)),
            LineIntersection::Overlap(xy(0, 0), xy(1, 1))
        );
        assert_eq!(
            intersection(xy(1, 1), xy(0, 0), xy(0, 0), xy(1, 1)),
            LineIntersection::Overlap(xy(1, 1), xy(0, 0))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 1), xy(2, 2)),
            LineIntersection::Point(xy(1, 1))
        );
        assert_eq!(
            intersection(xy(1, 1), xy(0, 0), xy(1, 1), xy(2, 2)),
            LineIntersection::Point(xy(1, 1))
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(2, 2), xy(4, 4)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, -1), xy(1, 0)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(1, 1), xy(0, 0), xy(0, -1), xy(1, 0)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(0, -1), xy(1, 0), xy(0, 0), xy(1, 1)),
            LineIntersection::None
        );

        assert_eq!(
            intersection(xy(0, 0.5), xy(1, 1.5), xy(0, 1), xy(1, 0)),
            LineIntersection::Point(xy(0.25, 0.75))
        );
    }
}
