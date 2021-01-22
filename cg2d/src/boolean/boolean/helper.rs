use std::cmp::Ordering;

#[inline]
pub fn less_if(condition: bool) -> Ordering {
    if condition {
        Ordering::Less
    } else {
        Ordering::Greater
    }
}

#[cfg(test)]
pub mod test {
    use nalgebra::Point2;

    pub fn xy<X: Into<f64>, Y: Into<f64>>(x: X, y: Y) -> Point2<f64> {
        Point2::new(x.into(), y.into())
    }
}
