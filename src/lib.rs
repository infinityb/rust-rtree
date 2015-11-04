#![allow(dead_code)]
mod bbox;
mod vec3;
mod ray;

use std::slice::Iter as SliceIter;
use std::f64;
pub use ray::Ray;
pub use bbox::{BBox};

#[cfg(test)]
mod test_helpers;

// References:
//  * http://www.cs.sfu.ca/CourseCentral/454/jpei/slides/R-Tree.pdf
//  * http://www-db.deis.unibo.it/courses/SI-LS/papers/Gut84.pdf
//  * http://avid.cs.umass.edu/courses/645/s2007/lectures/CS645-Lec9-RTree.pdf

const NODE_SIZE: usize = 64;

pub trait Mbr: Sized {
    fn mbr(&self) -> BBox;
}

#[must_use]
#[derive(PartialEq, PartialOrd, Eq, Ord, Debug, Hash)]
/// Represents the result of an Insertion: either the item fit, or the node had to split
pub enum InsertionResult<T> { 
    /// The inserted element fit and the bounding box was not changed.
    Fit,

    /// The inserted element fit and the bounding box was changed.
    Expanded,

    /// The inserted element did not fit, so the node was split.  Re-insert 
    /// the following items.
    Split(Vec<T>),
}

struct LeafItem<T> {
    bbox: BBox,
    item: T,
}

impl<T> LeafItem<T> where T: Mbr {
    fn new(item: T) -> LeafItem<T> {
        let bbox = item.mbr();
        LeafItem {
            bbox: bbox,
            item: item,
        }
    }
}

enum NodeStorage<T> where T: Mbr {
    Interior(Vec<RTreeNode<T>>),
    Leaf(Vec<LeafItem<T>>),
}

impl<T> NodeStorage<T> where T: Mbr {
    pub fn new_leaf_node(item: T) -> NodeStorage<T> {
        NodeStorage::Leaf(vec![LeafItem::new(item)])
    }

    pub fn shallow_len(&self) -> usize {
        match *self {
            NodeStorage::Interior(ref vec) => vec.len(),
            NodeStorage::Leaf(ref vec) => vec.len(),
        }
    }

    pub fn deep_len(&self) -> usize {
        match *self {
            NodeStorage::Interior(ref vec) => {
                vec.iter().map(|v| v.deep_len()).fold(0, |a,x| a+x)
            },
            NodeStorage::Leaf(ref vec) => vec.len(),
        }
    }
}

struct RTreeNode<T> where T: Mbr {
    bbox: BBox,
    storage: NodeStorage<T>,
}

impl<T> Mbr for RTreeNode<T> where T: Mbr {
    fn mbr(&self) -> BBox {
        self.bbox
    }
}

impl<T> RTreeNode<T> where T: Mbr {
    pub fn new(item: T) -> RTreeNode<T> {
        let bbox = item.mbr();

        RTreeNode {
            bbox: bbox,
            storage: NodeStorage::new_leaf_node(item),
        }
    }

    pub fn is_full(&self) -> bool {
        NODE_SIZE <= self.storage.shallow_len()
    }

    pub fn shallow_len(&self) -> usize {
        self.storage.shallow_len()
    }

    pub fn deep_len(&self) -> usize {
        self.storage.deep_len()
    }

    pub fn split(&mut self) -> RTreeNode<T> {
        match self.storage {
            NodeStorage::Interior(ref mut children) => {
                let (lbox, lefts, rbox, rights) =
                    util::quad_split(::std::mem::replace(children, Vec::new()));

                self.bbox = lbox;
                drop(::std::mem::replace(children, lefts));

                unimplemented!();
            },
            NodeStorage::Leaf(ref mut nodes) => {
                unimplemented!();
            }
        };
    }

    pub fn insert(&mut self, item: T) -> InsertionResult<T> {
        let item_bbox = item.mbr();

        match (self.is_full(), &mut self.storage) {
            (false, &mut NodeStorage::Interior(ref mut children)) => {
                let best_child = match util::best_fit(item_bbox, &children) {
                    Some(best_child) => best_child,
                    None => unimplemented!(),
                };
                match children[best_child].insert(item) {
                    InsertionResult::Fit => InsertionResult::Fit,
                    InsertionResult::Expanded => {
                        unimplemented!(); // recompute bounding box
                        // if we changed, emit Expanded, otherwise emit Fit.
                        InsertionResult::Expanded
                    },
                    InsertionResult::Split(items) => {
                        unimplemented!();
                    },
                }
            },
            (false, &mut NodeStorage::Leaf(ref mut nodes)) => {
                self.bbox = self.bbox.union(&item.mbr());
                nodes.push(LeafItem::new(item));
                InsertionResult::Fit
            },
            (true, _) => InsertionResult::Split(vec![item])
        }
    }
}

pub struct RTree<T>(Option<RTreeNode<T>>) where T: Mbr;

impl<T> RTree<T> where T: Mbr {
    pub fn new() -> RTree<T> {
        RTree(None)
    }

    pub fn insert(&mut self, item: T) {
        if self.0.is_none() {
            self.0 = Some(RTreeNode::new(item));
            return;
        }

        let mut node = self.0.take().unwrap();

        self.0 = Some(match node.insert(item) {
            InsertionResult::Expanded => {
                // recompute bounding box
                unimplemented!();
            },
            InsertionResult::Split(ref t) => {
                // make a new root?
                unimplemented!();
            }
            InsertionResult::Fit => node,
        });
    }

    pub fn iter_ray<'a>(&'a self, ray: &'a Ray) -> Iter<'a, T> {
        Iter::new(self, ray)
    }
}

pub struct Iter<'a, T> where T: Mbr+'a{
    stack: Vec<&'a RTreeNode<T>>,
    leaf_iter: Option<SliceIter<'a, LeafItem<T>>>,
    ray: &'a Ray,
}

impl<'a, T> Iter<'a, T> where T: Mbr+'a {
    fn new(rtree: &'a RTree<T>, ray: &'a Ray) -> Iter<'a, T> {
        let mut stack: Vec<&'a RTreeNode<T>> = Vec::new();
        if let Some(ref root) = rtree.0 {
            stack.push(root);
        }
        Iter {
            stack: stack,
            leaf_iter: None,
            ray: ray,
        }
    }
}

impl<'a, T> Iterator for Iter<'a, T> where T: Mbr+'a {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        loop {
            let ray = self.ray;
            if let Some(leaf_iter) = self.leaf_iter.as_mut() {
                if let Some(val) = leaf_iter.filter(|x| x.bbox.intersects(ray)).next() {
                    return Some(&val.item);
                }
            }

            // iterator went empty, so we'll pop from the stack and
            // iterate on the next node's children now,
            if let Some(node) = self.stack.pop() {
                match node.storage {
                    NodeStorage::Interior(ref children) => {
                        for child in children.iter() {
                            if child.bbox.intersects(self.ray) {
                                self.stack.push(child);
                            }
                        }
                    }
                    NodeStorage::Leaf(ref items) => {
                        self.leaf_iter = Some(items.iter())
                    }
                }
            } else {
                return None;
            }
        }
    }
}

mod util {
    use std::f64;
    use bbox::{BBox};
    use std::cmp::{Ord, Ordering};
    use super::Mbr;

    fn pick_seeds<I>(children: I) -> Option<(BBox, BBox)>
        where
            I: Iterator<Item=(BBox, BBox)> {

        let mut min_d = f64::MAX;
        let mut best_box: Option<(BBox, BBox)> = None;

        for (e1, e2) in children {
            let difference = e1.union(&e2).volume() - e1.volume() - e2.volume();
            if difference < min_d {
                min_d = difference;
                best_box = Some((e1, e2));
            }
        }

        best_box
    }

    fn expansion(target: &BBox, adding: &BBox) -> f64 {
        target.union(adding).volume() - target.volume()
    }

    pub fn quad_split<T>(items: Vec<T>) -> (BBox, Vec<T>, BBox, Vec<T>)
        where
            T: Mbr {

        let (mut lbox, mut rbox) = pick_seeds(Iterator::zip(
            items.iter().map(Mbr::mbr),
            items.iter().map(Mbr::mbr),
        )).expect("Unsufficient nodes");

        // `items` should be the size of a full node.  Size the other two
        // similarly.
        let mut lefts = Vec::with_capacity(items.len());
        let mut rights = Vec::with_capacity(items.len());

        // TODO: We need to select the best child first, instead of doing them
        // sequentially.
        for item in items.into_iter() {
            let ibox = item.mbr();

            let comparison = PartialOrd::partial_cmp(
                &expansion(&lbox, &ibox),
                &expansion(&rbox, &ibox),
            ).expect("Failed to compare box expansions");

            let (children, bbox) = match comparison {
                Ordering::Less => (&mut lefts, &mut lbox),
                Ordering::Equal => match Ord::cmp(&lefts.len(), &rights.len()) {
                    Ordering::Less => (&mut lefts, &mut lbox),
                    Ordering::Equal => (&mut lefts, &mut lbox),
                    Ordering::Greater => (&mut rights, &mut rbox),
                },
                Ordering::Greater => (&mut rights, &mut rbox),
            };
            *bbox = item.mbr().union(bbox);
            children.push(item);
        }
        (lbox, lefts, rbox, rights)
    }

    pub fn best_fit<T>(target: BBox, children: &[T]) -> Option<usize> where T: Mbr {

        if children.len() == 0 {
            return None;
        }

        let mut min_volume = f64::MAX;
        let mut best_idx = None;

        // Find the smallest node that contains our target.
        for (idx, child) in children.iter().enumerate() {
            let bbox = child.mbr();
            let volume = bbox.volume();
            if volume.is_nan() {
                panic!("volume must not be NaN");
            }
            if bbox.contains(&target) && volume < min_volume {
                min_volume = volume;
                best_idx = Some(idx);
            }
        }

        if let Some(idx) = best_idx {
            return Some(idx);
        }

        // None of the candidates fully contained our target, so search for the
        // node which would expand the least, if our target was added to it.
        for (idx, child) in children.iter().enumerate() {
            let volume = child.mbr().union(&target).volume();
            if volume.is_nan() {
                panic!("volume must not be NaN");
            }
            if volume < min_volume {
                min_volume = volume;
                best_idx = Some(idx);
            }
        }

        if let Some(idx) = best_idx {
            return Some(idx);
        }

        unreachable!("a node must have been selected");
    }
}

#[cfg(test)]
mod tests {
    use ::vec3::Vec3;
    use ::ray::Ray;
    use super::RTree;
    use super::test_helpers::Sphere;

    #[test]
    fn test_sphere() {
        let ray = Ray::new(Vec3::xyz(0.0, 0.0, 0.0), Vec3::xyz(1.0, 0.055, 0.00));

        let mut spheres: RTree<Sphere> = RTree::new();
        spheres.insert(Sphere::new(Vec3::xyz(100.0, 0.0, 0.0), 5.0).unwrap());
        spheres.insert(Sphere::new(Vec3::xyz(120.0, 0.0, 0.0), 15.0).unwrap());
        spheres.insert(Sphere::new(Vec3::xyz(140.0, 0.0, 0.0), 25.0).unwrap());
        spheres.insert(Sphere::new(Vec3::xyz(160.0, 0.0, 0.0), 35.0).unwrap());
        spheres.insert(Sphere::new(Vec3::xyz(180.0, 0.0, 0.0), 45.0).unwrap());
        spheres.insert(Sphere::new(Vec3::xyz(200.0, 0.0, 0.0), 55.0).unwrap());
        let intersect_res = spheres.intersects(&ray);
        println!("intersect = {:?}", intersect_res);
        panic!();
    }
}

#[no_mangle]
pub extern "C" fn debugger() {}