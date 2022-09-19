use std::{
    cmp::Ord, collections::VecDeque, convert::TryInto, fmt::Debug, marker::PhantomData,
    num::Wrapping,
};

use num::{traits::NumAssign, Bounded, NumCast, Unsigned};

use bitvec::prelude::*;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Handle<IndexT = u32, SerialT = u32>
where
    IndexT: Unsigned + Default + Copy + Bounded + NumCast + NumAssign + Ord + TryFrom<usize>,
    <IndexT as TryFrom<usize>>::Error: Debug,
    usize: TryFrom<IndexT>,
    <usize as TryFrom<IndexT>>::Error: Debug,
    SerialT: Unsigned + Default + Copy,
{
    pub(self) index: IndexT,
    pub(self) serial: SerialT,
}

impl<IndexT, SerialT> Handle<IndexT, SerialT>
where
    IndexT: Unsigned + Default + Copy + Bounded + NumCast + NumAssign + Ord + TryFrom<usize>,
    <IndexT as TryFrom<usize>>::Error: Debug,
    usize: TryFrom<IndexT>,
    <usize as TryFrom<IndexT>>::Error: Debug,
    SerialT: Unsigned + Default + Copy,
{
    pub fn index(&self) -> IndexT {
        self.index
    }

    pub fn index_usize(&self) -> usize {
        self.index.try_into().unwrap()
    }

    pub fn serial(&self) -> SerialT {
        self.serial
    }
}

#[derive(Debug, Clone, Default)]
pub struct HandlePool<T, IndexT = u32, SerialT = u32>
where
    T: Default,
    IndexT: Unsigned + Default + Copy + Bounded + NumCast + NumAssign + Ord + TryFrom<usize>,
    <IndexT as TryFrom<usize>>::Error: Debug,
    usize: TryFrom<IndexT>,
    <usize as TryFrom<IndexT>>::Error: Debug,
    SerialT: Unsigned + Default + Copy + NumAssign,
    Wrapping<SerialT>: NumAssign,
{
    data: Vec<T>,
    serials: Vec<Wrapping<SerialT>>,
    free_list: VecDeque<IndexT>,
    is_active: BitVec,
    index_t: PhantomData<IndexT>,
    serial_t: PhantomData<SerialT>,
}

impl<T, IndexT, SerialT> HandlePool<T, IndexT, SerialT>
where
    T: Default,
    IndexT: Unsigned + Default + Copy + Bounded + NumCast + NumAssign + Ord + TryFrom<usize>,
    <IndexT as TryFrom<usize>>::Error: Debug,
    usize: TryFrom<IndexT>,
    <usize as TryFrom<IndexT>>::Error: Debug,
    SerialT: Unsigned + Default + Copy + NumAssign,
    Wrapping<SerialT>: NumAssign,
{
    #[inline]
    pub fn clear(&mut self) {
        for i in 0..self.data.len() {
            if !self.is_active[i] {
                self.free_list.push_back(i.try_into().unwrap());
                self.serials[i] += Wrapping(SerialT::one());
                self.is_active.set(i, false);
            }
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[inline]
    pub fn capacity(&self) -> usize {
        self.data.capacity()
    }

    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        assert!(
            additional <= IndexT::max_value().try_into().unwrap(),
            "Max Possible Capacity Reached"
        );
        self.data.reserve(additional);
        self.serials.reserve(additional);
        self.free_list.reserve(additional);
    }

    #[inline]
    pub fn is_handle_valid(&self, handle: &Handle<IndexT, SerialT>) -> bool {
        let index_usize = handle.index_usize();
        if index_usize >= self.data.len() {
            return false;
        }
        self.is_active[index_usize] && self.serials[index_usize].0 == handle.serial()
    }

    pub fn add(&mut self) -> (&mut T, Handle<IndexT, SerialT>) {
        if self.free_list.is_empty() {
            assert!(
                TryInto::<IndexT>::try_into(self.free_list.len()).unwrap() <= IndexT::max_value(),
                "Max Possible Capacity Reached"
            );
            self.data.push(Default::default());
            self.serials.push(Default::default());
            self.is_active.push(false);
            let prev_len = self.free_list.len().try_into().unwrap();
            self.free_list.push_back(prev_len);
        }

        let free_pos = self.free_list.pop_front().unwrap();
        let free_pos_usize = free_pos.to_usize().unwrap();

        let resp = (
            &mut self.data[free_pos_usize],
            Handle {
                index: free_pos,
                serial: self.serials[free_pos_usize].0,
            },
        );
        self.is_active.set(free_pos_usize, true);

        resp
    }

    pub fn remove(&mut self, handle: &Handle<IndexT, SerialT>) -> bool {
        let index_usize = handle.index_usize();
        if index_usize >= self.data.len() {
            return false;
        }

        if !self.is_active[index_usize] {
            // Inactive object
            return false;
        }

        let serial = self.serials[index_usize];
        if serial.0 != handle.serial() {
            // Wrong serial, trying to delete a previous object
            return false;
        }

        assert!(!self.is_empty(), "The pool is empty");

        self.serials[index_usize] += Wrapping(SerialT::one());
        self.is_active.set(index_usize, false);

        self.free_list.push_back(handle.index());

        true
    }

    pub fn get(&self, handle: &Handle<IndexT, SerialT>) -> Option<&T> {
        let index_usize = handle.index_usize();
        if index_usize >= self.data.len() {
            return None;
        }

        if !self.is_active[index_usize] {
            // Inactive object
            return None;
        }

        let serial = self.serials[index_usize];
        if serial.0 != handle.serial() {
            // Wrong serial, trying to delete a previous object
            return None;
        }

        Some(&self.data[index_usize])
    }

    pub fn get_by_index(&self, index: IndexT) -> Option<&T> {
        let index_usize: usize = index.try_into().unwrap();
        if index_usize >= self.data.len() {
            return None;
        }

        if !self.is_active[index_usize] {
            // Inactive object
            return None;
        }

        Some(&self.data[index_usize])
    }

    pub fn get_mut(&mut self, handle: &Handle<IndexT, SerialT>) -> Option<&mut T> {
        let index_usize = handle.index_usize();
        if index_usize >= self.data.len() {
            return None;
        }

        if !self.is_active[index_usize] {
            // Inactive object
            return None;
        }

        let serial = self.serials[index_usize];
        if serial.0 != handle.serial() {
            // Wrong serial, trying to delete a previous object
            return None;
        }

        Some(&mut self.data[index_usize])
    }

    pub fn get_mut_by_index(&mut self, index: IndexT) -> Option<&mut T> {
        let index_usize: usize = index.try_into().unwrap();
        if index_usize >= self.data.len() {
            return None;
        }

        if !self.is_active[index_usize] {
            // Inactive object
            return None;
        }

        Some(&mut self.data[index_usize])
    }
}
