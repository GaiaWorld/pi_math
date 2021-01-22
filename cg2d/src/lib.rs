extern crate slab;
extern crate heap;
extern crate triangulation;
extern crate num_traits;
extern crate fx_hashmap;

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;

#[cfg(feature = "rstar")]
extern crate rstar;

/** 
 * geo2d，2D几何类型，geo-types
 *    + 基本形状：点，线段，多边形
 *    + github: https://github.com/georust/geo
 *    + 重点是：带孔多边形的描述
 * boolean：多边形的布尔运算：&，|，^
 *    + github: https://github.com/21re/rust-geo-booleanop
 */

mod util;
mod geo2d;
mod boolean;

pub use util::*;
pub use geo2d::*;