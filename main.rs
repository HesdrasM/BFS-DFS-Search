use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

// Tipo para representar o labirinto como uma matriz 2D
type Maze = Vec<Vec<u8>>;

// Estrutura para representar um ponto na matriz
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct Point {
    x: usize,
    y: usize,
}

// Direções possíveis para movimento: cima, baixo, esquerda, direita
const DIRECTIONS: [(isize, isize); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

// Função auxiliar para reconstruir o caminho a partir do mapa de pais
fn reconstruct_path(parent_map: &HashMap<Point, Option<Point>>, mut current: Point) -> Vec<Point> {
    let mut path = Vec::new();
    while let Some(parent) = parent_map.get(&current) {
        path.push(current);
        if let Some(p) = parent {
            current = *p;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

// Função de Busca em Largura (BFS)
fn bfs(maze: &Maze, start: Point, goal: Point) -> Option<Vec<Point>> {
    let mut queue = VecDeque::new(); // Fila para BFS
    let mut visited = HashSet::new(); // Conjunto de pontos visitados
    let mut parent_map = HashMap::new(); // Mapa de pais para reconstruir o caminho

    // Inicializa a fila com o ponto de início
    queue.push_back(start);
    visited.insert(start);
    parent_map.insert(start, None);

    while let Some(current) = queue.pop_front() {
        // Se o ponto atual é o objetivo, reconstrua e retorne o caminho
        if current == goal {
            return Some(reconstruct_path(&parent_map, current));
        }

        // Explora os vizinhos do ponto atual
        for &(dx, dy) in &DIRECTIONS {
            let new_x = (current.x as isize + dx) as usize;
            let new_y = (current.y as isize + dy) as usize;

            // Verifica se o novo ponto é válido e não é uma parede
            if new_x < maze.len() && new_y < maze[0].len() && maze[new_x][new_y] == 0 {
                let new_point = Point { x: new_x, y: new_y };
                if !visited.contains(&new_point) {
                    visited.insert(new_point);
                    queue.push_back(new_point);
                    parent_map.insert(new_point, Some(current));
                }
            }
        }
    }

    None
}

// Função de Busca em Profundidade (DFS)
fn dfs(maze: &Maze, start: Point, goal: Point) -> Option<Vec<Point>> {
    let mut stack = Vec::new(); // Pilha para DFS
    let mut visited = HashSet::new(); // Conjunto de pontos visitados
    let mut parent_map = HashMap::new(); // Mapa de pais para reconstruir o caminho

    // Inicializa a pilha com o ponto de início
    stack.push(start);
    visited.insert(start);
    parent_map.insert(start, None);

    while let Some(current) = stack.pop() {
        // Se o ponto atual é o objetivo, reconstrua e retorne o caminho
        if current == goal {
            return Some(reconstruct_path(&parent_map, current));
        }

        // Explora os vizinhos do ponto atual
        for &(dx, dy) in &DIRECTIONS {
            let new_x = (current.x as isize + dx) as usize;
            let new_y = (current.y as isize + dy) as usize;

            // Verifica se o novo ponto é válido e não é uma parede
            if new_x < maze.len() && new_y < maze[0].len() && maze[new_x][new_y] == 0 {
                let new_point = Point { x: new_x, y: new_y };
                if !visited.contains(&new_point) {
                    visited.insert(new_point);
                    stack.push(new_point);
                    parent_map.insert(new_point, Some(current));
                }
            }
        }
    }

    None
}

// Estrutura para representar um nó na busca gulosa
#[derive(Eq, PartialEq)]
struct Node {
    point: Point,
    heuristic: usize,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.heuristic.cmp(&self.heuristic) // Inversão para criar um min-heap
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Função de Busca Gulosa (Greedy Best-First Search)
fn greedy_best_first_search(maze: &Maze, start: Point, goal: Point) -> Option<Vec<Point>> {
    let mut open_set = BinaryHeap::new(); // Fila de prioridade (min-heap)
    let mut visited = HashSet::new(); // Conjunto de pontos visitados
    let mut parent_map = HashMap::new(); // Mapa de pais para reconstruir o caminho

    // Função heurística para calcular a distância de Manhattan
    let heuristic = |point: Point| -> usize {
        ((point.x as isize - goal.x as isize).abs() + (point.y as isize - goal.y as isize).abs())
            as usize
    };

    // Inicializa a fila de prioridade com o ponto de início
    open_set.push(Node {
        point: start,
        heuristic: heuristic(start),
    });
    visited.insert(start);
    parent_map.insert(start, None);

    while let Some(Node { point: current, .. }) = open_set.pop() {
        // Se o ponto atual é o objetivo, reconstrua e retorne o caminho
        if current == goal {
            return Some(reconstruct_path(&parent_map, current));
        }

        // Explora os vizinhos do ponto atual
        for &(dx, dy) in &DIRECTIONS {
            let new_x = (current.x as isize + dx) as usize;
            let new_y = (current.y as isize + dy) as usize;

            // Verifica se o novo ponto é válido e não é uma parede
            if new_x < maze.len() && new_y < maze[0].len() && maze[new_x][new_y] == 0 {
                let new_point = Point { x: new_x, y: new_y };
                if !visited.contains(&new_point) {
                    visited.insert(new_point);
                    open_set.push(Node {
                        point: new_point,
                        heuristic: heuristic(new_point),
                    });
                    parent_map.insert(new_point, Some(current));
                }
            }
        }
    }

    None
}

// Estrutura para representar um nó na busca A*
#[derive(Eq, PartialEq)]
struct AStarNode {
    point: Point,
    cost: usize,
    heuristic: usize,
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.cost + self.heuristic).cmp(&(other.cost + other.heuristic)) // Soma do custo e heurística para priorizar
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Função de Busca A*
fn a_star(maze: &Maze, start: Point, goal: Point) -> Option<Vec<Point>> {
    let mut open_set = BinaryHeap::new(); // Fila de prioridade (min-heap)
    let mut visited = HashSet::new(); // Conjunto de pontos visitados
    let mut parent_map = HashMap::new(); // Mapa de pais para reconstruir o caminho
    let mut cost_map = HashMap::new(); // Mapa de custos acumulados

    // Função heurística para calcular a distância de Manhattan
    let heuristic = |point: Point| -> usize {
        ((point.x as isize - goal.x as isize).abs() + (point.y as isize - goal.y as isize).abs())
            as usize
    };

    // Inicializa a fila de prioridade com o ponto de início
    open_set.push(AStarNode {
        point: start,
        cost: 0,
        heuristic: heuristic(start),
    });
    visited.insert(start);
    parent_map.insert(start, None);
    cost_map.insert(start, 0);

    while let Some(AStarNode {
        point: current,
        cost,
        ..
    }) = open_set.pop()
    {
        // Se o ponto atual é o objetivo, reconstrua e retorne o caminho
        if current == goal {
            return Some(reconstruct_path(&parent_map, current));
        }

        // Explora os vizinhos do ponto atual
        for &(dx, dy) in &DIRECTIONS {
            let new_x = (current.x as isize + dx) as usize;
            let new_y = (current.y as isize + dy) as usize;

            // Verifica se o novo ponto é válido e não é uma parede
            if new_x < maze.len() && new_y < maze[0].len() && maze[new_x][new_y] == 0 {
                let new_point = Point { x: new_x, y: new_y };
                let new_cost = cost + 1; // Custo para mover-se para o novo ponto

                if !visited.contains(&new_point)
                    || new_cost < *cost_map.get(&new_point).unwrap_or(&usize::MAX)
                {
                    visited.insert(new_point);
                    parent_map.insert(new_point, Some(current));
                    cost_map.insert(new_point, new_cost);
                    open_set.push(AStarNode {
                        point: new_point,
                        cost: new_cost,
                        heuristic: heuristic(new_point),
                    });
                }
            }
        }
    }

    None
}

// Função para exibir o labirinto com o caminho
fn print_maze_with_path(maze: &Maze, path: &[Point]) {
    // Cria uma cópia do labirinto para exibir o caminho
    let mut maze_with_path = maze.clone();

    // Marca o caminho no labirinto com 2
    for point in path {
        if point.x < maze_with_path.len() && point.y < maze_with_path[0].len() {
            maze_with_path[point.x][point.y] = 2;
        }
    }

    // Imprime o labirinto
    for row in maze_with_path.iter() {
        for &cell in row.iter() {
            let symbol = match cell {
                0 => ' ', // Caminho livre
                1 => '#', // Parede
                2 => '*', // Caminho encontrado
                _ => '?',
            };
            print!("{} ", symbol);
        }
        println!();
    }
}

fn main() {
    // Define o labirinto
    let maze: Maze = vec![
        vec![0, 1, 0, 0, 0],
        vec![0, 1, 0, 1, 0],
        vec![0, 0, 0, 1, 0],
        vec![0, 1, 0, 0, 0],
        vec![0, 0, 0, 1, 0],
    ];

    // Define o ponto de início e o objetivo
    let start = Point { x: 0, y: 0 };
    let goal = Point { x: 4, y: 4 };

    // Executa a BFS
    if let Some(path) = bfs(&maze, start, goal) {
        println!("BFS Path:");
        print_maze_with_path(&maze, &path);
        println!();
    }

    // Executa a DFS
    if let Some(path) = dfs(&maze, start, goal) {
        println!("DFS Path:");
        print_maze_with_path(&maze, &path);
        println!();
    }

    // Executa a Busca Gulosa
    if let Some(path) = greedy_best_first_search(&maze, start, goal) {
        println!("Greedy Best-First Search Path:");
        print_maze_with_path(&maze, &path);
        println!();
    }

    // Executa a Busca A*
    if let Some(path) = a_star(&maze, start, goal) {
        println!("A* Path:");
        print_maze_with_path(&maze, &path);
        println!();
    }
}
