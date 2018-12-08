use std::collections::{HashMap, BinaryHeap, VecDeque};
use std::cmp::Ordering;
use std::fmt::Debug;
use std::hash::Hash;
const WIDTH:usize = 4;


struct DistanceAndPrev<VertexT> {
    distance: f64,
    prev: Option<VertexT>
}

#[derive(Clone, PartialEq)]
struct VertexAndDist<VertexT> {
    vert: VertexT,
    heuristic: f64,
    dist: f64,
    total: f64,
}

impl<T> Ord for VertexAndDist<T> where T : Ord + PartialEq  + PartialOrd {
    fn cmp(&self, rhs:&Self) -> Ordering {
        rhs.partial_cmp(&self).unwrap()
            .then_with(|| rhs.dist.partial_cmp(&self.dist).unwrap())
            .then_with(|| rhs.vert.partial_cmp(&self.vert).unwrap())
            .then_with(|| rhs.heuristic.partial_cmp(&self.heuristic).unwrap())
    }
}

impl<T> PartialOrd for VertexAndDist<T> where T : Ord + PartialEq {
    fn partial_cmp(&self, rhs:&Self) -> Option<Ordering> {
        rhs.total.partial_cmp(&self.total)
    }
}


fn pretty_print_path(path:&VecDeque<PuzzleState>) {
    let slow = path.iter();
    let fast = slow.clone().skip(1);

    println!("Solution found in {} steps", path.len()-1);
    for (slow, fast) in slow.zip(fast) {
        let fast_empty_i = fast.sqs.iter().position(Option::is_none).unwrap();
        print!("{:?}  ", slow.sqs[fast_empty_i].unwrap());
    }
}

fn unwind_into_path<VertexT>(hm:&HashMap<VertexT, DistanceAndPrev<VertexT>>, goal_node:&VertexT) -> VecDeque<VertexT> where VertexT : Debug + Hash + Eq + Clone {
    let mut cur:Option<&VertexT> = Some(goal_node);
    let mut vd = VecDeque::new();
    while let Some(v_ref) = cur {
        cur = hm.get(v_ref).unwrap().prev.as_ref();
        vd.push_front(v_ref.clone());
    }
    vd
}




impl<T> Eq for VertexAndDist<T> where T : Ord + PartialEq {}

fn a_star<'a, VertexT : 'static, AdjFn: 'a, HerFn : 'a>(start_node: &'a VertexT, goal_node: &'a VertexT, adj_fn: AdjFn, heuristic_fn:HerFn) -> VecDeque<VertexT>
    where VertexT : Ord + PartialEq + Eq + Clone + Debug + Hash,
          AdjFn : Fn(&VertexT) -> Vec<VertexT>,
          HerFn : Fn(&VertexT) -> f64,
{

    let mut heap:BinaryHeap<VertexAndDist<VertexT>> = BinaryHeap::new();
    let mut dists_and_prevs:HashMap<VertexT, DistanceAndPrev<VertexT>> = HashMap::new();


    heap.push(VertexAndDist {vert : start_node.clone(), dist: 0_f64, heuristic:0_f64, total:0_f64});
    dists_and_prevs.insert(start_node.clone(), DistanceAndPrev { distance: 0_f64, prev: None});

    while let Some(VertexAndDist {vert:parent_vert, dist: parent_distance, heuristic: parent_heuristic, total: parent_total}) = heap.pop() {
//        println!("d={:?}", parent_distance);
        if &parent_vert == goal_node {
            return unwind_into_path(&dists_and_prevs, &goal_node);
        }
        let children = adj_fn(&parent_vert);

        for child in children.into_iter() {
            let her = heuristic_fn(&child);
            let distance = 1_f64 + parent_distance;
            let existing_distance = dists_and_prevs.get(&child).map(|d_and_p| d_and_p.distance).unwrap_or(std::f64::INFINITY);
            if distance < existing_distance {
                dists_and_prevs.insert(child.clone(), DistanceAndPrev {distance, prev: Some(parent_vert.clone())});
                heap.push(VertexAndDist {vert : child, dist :distance, heuristic: her, total: distance + her});
            }
        }
    }
    panic!("No path")

}



#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct PuzzleState {
    sqs: [Option<u8>;WIDTH*WIDTH]
}

impl std::fmt::Debug for PuzzleState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        for row_i in 0..WIDTH {
            let row = &self.sqs[row_i*WIDTH..row_i*WIDTH+WIDTH];
            let line = row.iter().map(|cell| {
                match cell {
                    &Some(n) => format!("{:2}", n),
                    &None => String::from("  ")
                }
            }).collect::<Vec<_>>().join(" | ");
            write!(f, "\n{}", line)?;
        }
       Result::Ok(())
    }
}

impl PuzzleState {
    fn from_slice(slice:&[u8]) -> PuzzleState {
        let mut new_slice = [Some(0_u8); WIDTH*WIDTH];
        for (i, &el) in slice.iter().enumerate() {
            new_slice[i] = Some(el);
        }
        PuzzleState {sqs: new_slice}
    }
    fn x_y_to_index(x:usize, y:usize) -> usize {
        (y * WIDTH) + x
    }

    fn index_to_xy(index:usize) -> (usize, usize) {
        let y = index / WIDTH;
        let x = index - (y * WIDTH);
        (x,y)
    }

    fn set_square(&mut self, x:usize, y:usize, val:u8) {
        let index = Self::x_y_to_index(x, y);
        self.sqs[index] = Some(val);
    }

    fn clear_square(&mut self, x:usize, y:usize) {
        let index = Self::x_y_to_index(x, y);
        self.sqs[index] = None;
        println!("{:?}", self.sqs);
    }


    // @Optimization, don't return a vec of puzzlestate, append to an exisiting one
    fn get_possible_moves(&self) -> Vec<PuzzleState> {
        // Presumes there is only empty square
        let i:i32 = self.sqs.iter().enumerate().find(|&(uz, it)| it.is_none()).unwrap().0 as i32;


        // Calculations are done in integer so that they can go negative, the negatives
        // later get filtered out

        let WIDTH_U = WIDTH as i32;
        let below_i = i + WIDTH_U;
        let above_i = i - WIDTH_U;
        let right_i = if (i+1) % WIDTH_U == 0 {-1} else {i+1};
        let left_i = if (i+1) % WIDTH_U == 1 {-1} else {i-1} ;

        let indices:[i32;4] = [below_i, above_i, right_i, left_i];


        let copies = indices.iter().filter(|&&ind| ind >= 0 && ind < WIDTH_U * WIDTH_U).map(|&adj_i| {
            let mut clone_of_state = self.clone();
            let adj_val = self.sqs[adj_i as usize].unwrap();

            assert_eq!(clone_of_state.sqs[i as usize], None, "Trying to shift into an occupied square!");
            clone_of_state.sqs[i as usize] = Some(adj_val);
            clone_of_state.sqs[adj_i as usize] = None;

            clone_of_state
        }).collect::<Vec<_>>();
        copies
    }


    fn hueristic_equal_indicies(&self) -> impl  Fn(&Self) -> f64 {
        let goal_state_clone = self.sqs.clone();
        let this_heuristic_function  = move |pz_state:&Self|{
            let correct_squares = goal_state_clone.iter().zip(pz_state.sqs.iter()).filter(|&(n1, n2)| n1 == n2).count();
            (WIDTH * WIDTH - correct_squares) as f64
        };

        this_heuristic_function
    }

    fn heuristic_manhattan_distance(&self) -> impl Fn(&Self) -> f64 {
        let goal_state_clone = self.sqs.clone();

        let manh_heur = move |pz_state: &Self|  {
            goal_state_clone.iter().enumerate().map(|(i, sq_ref)| {
                let goal_xy = Self::index_to_xy(i);

                let other_index = pz_state.sqs
                    .iter()
                    .enumerate()
                    .find(|&(i, pz_sq_ref)| pz_sq_ref == sq_ref)
                    .expect("Couldn't find goal_sq in pz_sq")
                    .0;

                let other_xy = Self::index_to_xy(other_index);
                let dx = (goal_xy.0 as i32 - other_xy.0 as i32).abs();
                let dy = (goal_xy.1 as i32 - other_xy.1 as i32).abs();

                let total:i32 = dx + dy;
                total

            }).sum::<i32>() as f64
        };
        manh_heur
    }
}



struct Test {
    a:i32,
    b:i32,
}
fn main() {



    let mut start = PuzzleState::from_slice(&[3, 6, 0, 4, 1, 2, 7, 8, 14, 11, 10, 12, 13, 5, 9, 15]);
    start.clear_square(2, 0);

    let adjs = start.get_possible_moves();

//    println!("ADJS = {:#?}", adjs);


    let mut goal = PuzzleState::from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0]);
    goal.clear_square(3, 3);

    println!("START={:#?}", start);
    println!("GOAL={:#?}", goal);
    let s = std::time::Instant::now();


    let useless_heuristic = |_:&PuzzleState| 0_f64; // For playing around with how speed varies depending on heuristic
    let manhattan_heuristic = goal.heuristic_manhattan_distance();
    let equal_indicies = goal.hueristic_equal_indicies();
    let path = a_star(&start, &goal, PuzzleState::get_possible_moves, equal_indicies);

    let el = s.elapsed();


    println!("Found solution in {:?} seconds", el.as_secs() as f64 + el.subsec_nanos() as f64 / 1e9);
    pretty_print_path(&path);
}



