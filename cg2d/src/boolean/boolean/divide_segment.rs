use super::sweep_event::SweepEvent;
use heap::simple_heap::SimpleHeap;
use nalgebra::{Point2, RealField, Scalar};
use std::rc::Rc;

pub fn divide_segment<F: Scalar + RealField>(
    se: &Rc<SweepEvent<F>>,
    inter: Point2<F>,
    queue: &mut SimpleHeap<Rc<SweepEvent<F>>>,
) {
    let other_event = match se.get_other_event() {
        Some(other_event) => other_event,
        None => return,
    };

    let r = SweepEvent::new_rc(
        se.contour_id,
        inter,
        false,
        Rc::downgrade(&se),
        se.is_subject,
        true,
    );
    let l = SweepEvent::new_rc(
        se.contour_id,
        inter,
        true,
        Rc::downgrade(&other_event),
        se.is_subject,
        true,
    );

    if l < other_event {
        se.set_left(true);
        l.set_left(false);
    }

    other_event.set_other_event(&l);
    se.set_other_event(&r);

    queue.push(l);
    queue.push(r);
}

#[cfg(test)]
mod test {
    use super::super::segment_intersection::{intersection, LineIntersection};
    use super::super::sweep_event::SweepEvent;
    use super::*;
    use nalgebra::Point2;
    use std::cmp::Ordering;
    use std::rc::{Rc, Weak};

    fn make_simple(
        x: f64,
        y: f64,
        other_x: f64,
        other_y: f64,
        is_subject: bool,
    ) -> (Rc<SweepEvent<f64>>, Rc<SweepEvent<f64>>) {
        let other = SweepEvent::new_rc(
            0,
            Point2::new(other_x, other_y),
            false,
            Weak::new(),
            is_subject,
            true,
        );
        let event = SweepEvent::new_rc(
            0,
            Point2::new(x, y),
            true,
            Rc::downgrade(&other),
            is_subject,
            true,
        );

        (event, other)
    }

    #[test]
    fn devide_segments() {
        let (se1, other1) = make_simple(0.0, 0.0, 5.0, 5.0, true);
        let (se2, other2) = make_simple(0.0, 5.0, 5.0, 0.0, false);
        let mut queue = SimpleHeap::new(Ordering::Greater);

        queue.push(se1.clone());
        queue.push(se2.clone());

        let inter = match intersection(se1.point, other1.point, se2.point, other2.point) {
            LineIntersection::Point(p) => p,
            _ => panic!("Not a point intersection"),
        };

        divide_segment(&se1, inter, &mut queue);
        divide_segment(&se2, inter, &mut queue);

        assert_eq!(queue.len(), 6);
    }
}
