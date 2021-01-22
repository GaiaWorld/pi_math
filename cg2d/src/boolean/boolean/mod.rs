use crate::geo2d::{Polygon, Rectangle};
use nalgebra::{Point2, RealField, Scalar};
use num_traits::Float;
pub mod compare_segments;
pub mod compute_fields;
mod connect_edges;
mod divide_segment;
pub mod fill_queue;
mod helper;
pub mod possible_intersection;
mod segment_intersection;
mod signed_area;
pub mod subdivide_segments;
pub mod sweep_event;

use self::connect_edges::connect_edges;
use self::fill_queue::fill_queue;
use self::subdivide_segments::subdivide;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Operation {
    Intersection,
    Difference,
    Union,
    Xor,
}

pub trait BooleanOp<F, Rhs = Self>
where
    F: Float + Scalar + RealField,
{
    fn boolean(&self, rhs: &Rhs, operation: Operation) -> Vec<Polygon<F>>;

    fn intersection(&self, rhs: &Rhs) -> Vec<Polygon<F>> {
        self.boolean(rhs, Operation::Intersection)
    }

    fn difference(&self, rhs: &Rhs) -> Vec<Polygon<F>> {
        self.boolean(rhs, Operation::Difference)
    }

    fn union(&self, rhs: &Rhs) -> Vec<Polygon<F>> {
        self.boolean(rhs, Operation::Union)
    }

    fn xor(&self, rhs: &Rhs) -> Vec<Polygon<F>> {
        self.boolean(rhs, Operation::Xor)
    }
}

impl<F> BooleanOp<F> for Polygon<F>
where
    F: Float + Scalar + RealField,
{
    fn boolean(&self, rhs: &Polygon<F>, operation: Operation) -> Vec<Polygon<F>> {
        boolean_operation(&[self.clone()], &[rhs.clone()], operation)
    }
}

impl<F> BooleanOp<F, Vec<Polygon<F>>> for Polygon<F>
where
    F: Float + Scalar + RealField,
{
    fn boolean(&self, rhs: &Vec<Polygon<F>>, operation: Operation) -> Vec<Polygon<F>> {
        boolean_operation(&[self.clone()], rhs.as_slice(), operation)
    }
}

impl<F> BooleanOp<F> for Vec<Polygon<F>>
where
    F: Float + Scalar + RealField,
{
    fn boolean(&self, rhs: &Vec<Polygon<F>>, operation: Operation) -> Vec<Polygon<F>> {
        boolean_operation(self.as_slice(), rhs.as_slice(), operation)
    }
}

impl<F> BooleanOp<F, Polygon<F>> for Vec<Polygon<F>>
where
    F: Float + Scalar + RealField,
{
    fn boolean(&self, rhs: &Polygon<F>, operation: Operation) -> Vec<Polygon<F>> {
        boolean_operation(self.as_slice(), &[rhs.clone()], operation)
    }
}

fn boolean_operation<F>(
    subject: &[Polygon<F>],
    clipping: &[Polygon<F>],
    operation: Operation,
) -> Vec<Polygon<F>>
where
    F: Float + Scalar + RealField,
{
    let mut sbbox = Rectangle {
        mins: Point2::new(F::infinity(), F::infinity()),
        maxs: Point2::new(F::neg_infinity(), F::neg_infinity()),
    };
    let mut cbbox = sbbox;

    let mut event_queue = fill_queue(subject, clipping, &mut sbbox, &mut cbbox, operation);

    if sbbox.mins.x > cbbox.maxs.x
        || cbbox.mins.x > sbbox.maxs.x
        || sbbox.mins.y > cbbox.maxs.y
        || cbbox.mins.y > sbbox.maxs.y
    {
        return trivial_result(subject, clipping, operation);
    }

    let sorted_events = subdivide(&mut event_queue, &sbbox, &cbbox, operation);

    connect_edges(&sorted_events, operation)
}

fn trivial_result<F>(
    subject: &[Polygon<F>],
    clipping: &[Polygon<F>],
    operation: Operation,
) -> Vec<Polygon<F>>
where
    F: Float + Scalar + RealField,
{
    match operation {
        Operation::Intersection => vec![],
        Operation::Difference => Vec::from(subject),
        Operation::Union | Operation::Xor => subject.iter().chain(clipping).cloned().collect(),
    }
}
