package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

import algorithms.DefaultTeam.Arete;
import algorithms.DefaultTeam.PointIdent;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

public class DefaultTeam {
	public ArrayList<Point> calculConnectedDominatingSet(ArrayList<Point> points, int edgeThreshold) {
		// REMOVE >>>>>
		ArrayList<Point> result = (ArrayList<Point>) points.clone();
		for (int i = 0; i < points.size() / 3; i++)
			result.remove(0);
		// if (false) result = readFromFile("output0.points");
		// else saveToFile("output",result);
		// <<<<< REMOVE
		System.out.println(TreeToPoints(calculSteiner(points, edgeThreshold, maximalIndependentSet(points, edgeThreshold))));
		//System.out.println(IsValid(maximalIndependentSet(points, edgeThreshold), edgeThreshold ) );
		//System.out.println(points.size());
		//System.out.println(maximalIndependentSet(points, edgeThreshold).size());
		return TreeToPoints(calculSteiner(points, edgeThreshold, maximalIndependentSet(points, edgeThreshold)));
	}
	
	public ArrayList<Point> TreeToPoints(Tree2D tree){
		System.out.println("je rentre dedans");

		if (tree.getSubTrees() == null) {
			ArrayList<Point> points = new ArrayList<Point>();
			points.add(tree.getRoot());
			return points;
		}
		else {
			ArrayList<Point> points = new ArrayList<Point>();
			 points.add(tree.getRoot());
			 for(int i = 0; i<tree.getSubTrees().size();i++) {
				 System.out.println("b" + points.size());
				 points.addAll(TreeToPoints(tree.getSubTrees().get(i)));
				 System.out.println("a" + points.size());

			 }
			 return points;
		}
	}
	
	public ArrayList<Point> neighbor(Point p, ArrayList<Point> vertices, int edgeThreshold) {
		ArrayList<Point> result = new ArrayList<Point>();
		for (Point point : vertices)
			if (point.distance(p) < edgeThreshold && !point.equals(p))
				result.add((Point) point.clone());
		return result;
	}
	
	public boolean IsValid(ArrayList<Point> vertices, int edgeThreshold ) {	
		for(Point p : vertices) {
			if(neighbor(p, vertices, edgeThreshold).size() > 0) {
				return false;
			}
		}
		return true;
		
	}
	
	public ArrayList<Point> maximalIndependentSet(ArrayList<Point> vertices, int edgeThreshold){
		ArrayList<Point> currentVs = new ArrayList<>();
		for(Point p : vertices) {
			currentVs.add(p);
			if (!IsValid(currentVs, edgeThreshold)) {
				currentVs.remove(p);
			}
		}
		return currentVs;
	}


	// FILE PRINTER
	private void saveToFile(String filename, ArrayList<Point> result) {
		int index = 0;
		try {
			while (true) {
				BufferedReader input = new BufferedReader(
						new InputStreamReader(new FileInputStream(filename + Integer.toString(index) + ".points")));
				try {
					input.close();
				} catch (IOException e) {
					System.err.println(
							"I/O exception: unable to close " + filename + Integer.toString(index) + ".points");
				}
				index++;
			}
		} catch (FileNotFoundException e) {
			printToFile(filename + Integer.toString(index) + ".points", result);
		}
	}

	private void printToFile(String filename, ArrayList<Point> points) {
		try {
			PrintStream output = new PrintStream(new FileOutputStream(filename));
			int x, y;
			for (Point p : points)
				output.println(Integer.toString((int) p.getX()) + " " + Integer.toString((int) p.getY()));
			output.close();
		} catch (FileNotFoundException e) {
			System.err.println("I/O exception: unable to create " + filename);
		}
	}

	// FILE LOADER
	private ArrayList<Point> readFromFile(String filename) {
		String line;
		String[] coordinates;
		ArrayList<Point> points = new ArrayList<Point>();
		try {
			BufferedReader input = new BufferedReader(new InputStreamReader(new FileInputStream(filename)));
			try {
				while ((line = input.readLine()) != null) {
					coordinates = line.split("\\s+");
					points.add(new Point(Integer.parseInt(coordinates[0]), Integer.parseInt(coordinates[1])));
				}
			} catch (IOException e) {
				System.err.println("Exception: interrupted I/O.");
			} finally {
				try {
					input.close();
				} catch (IOException e) {
					System.err.println("I/O exception: unable to close " + filename);
				}
			}
		} catch (FileNotFoundException e) {
			System.err.println("Input file not found.");
		}
		return points;
	}

	public static class PointIdent {
		static int NEX_ID = 0;
		private int clusterId = NEX_ID++;
		private Point pt;

		public PointIdent(Point p) {
			pt = p;
		}

		public int getId() {
			return clusterId;
		}

		public Point getPoint() {
			return pt;
		}

		public String toString() {
			return pt.toString();
		}
	}

	public class Arete {

		private PointIdent p;
		private PointIdent p2;
		private double poids;

		public Arete(PointIdent p, PointIdent p2, double poids) {
			this.p = p;
			this.p2 = p2;
			this.poids = poids;
		}

		public PointIdent getP() {
			return p;
		}

		public PointIdent getQ() {
			return p2;
		}

		public double poids() {
			return this.poids;
		}

		public String toString() {
			return "p1 " + p + " p2 " + p2;
		}
	}

	public ArrayList<Object> floydWarshall(ArrayList<Point> points, int edgeThreshold) {

		int n = points.size();
		double[][] M = new double[n][n];

		int[][] R = new int[n][n];

		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				double dist = points.get(i).distance(points.get(j));
				M[i][j] = dist < edgeThreshold ? dist : Double.POSITIVE_INFINITY;
				R[i][j] = j;
				// System.out.println("R[i][j] vaut " + R[i][j]);
			}
		}

		for (int p = 0; p < n; p++) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					if (M[i][j] > M[i][p] + M[p][j]) {
						M[i][j] = M[i][p] + M[p][j];
						R[i][j] = R[i][p];
					}
				}
			}
		}
		ArrayList<Object> result = new ArrayList<>();
		result.add(M);
		result.add(R);
		return result;
	}

	public int[][] calculShortestPaths(ArrayList<Point> points, int edgeThreshold) {

		int n = points.size();
		double[][] M = new double[n][n];

		int[][] R = new int[n][n];

		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				double dist = points.get(i).distance(points.get(j));
				M[i][j] = dist < edgeThreshold ? dist : Double.POSITIVE_INFINITY;
				R[i][j] = j;
				// System.out.println("R[i][j] vaut " + R[i][j]);
			}
		}

		for (int p = 0; p < n; p++) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					if (M[i][j] > M[i][p] + M[p][j]) {
						M[i][j] = M[i][p] + M[p][j];
						R[i][j] = R[i][p];
					}
				}
			}
		}
		return R;
	}

	public ArrayList<Arete> kruskal(ArrayList<Point> points2, ArrayList<Point> points3, double[][] M) {

		ArrayList<Arete> aretes = new ArrayList<Arete>();

		ArrayList<PointIdent> points = new ArrayList<>();
		for (Point pt : points2) {
			points.add(new PointIdent(pt));
		}

		int cpt = 0;
		for (int i = 0; i < points.size(); i++) {
			for (int j = i + 1; j < points.size(); j++) {
				cpt++;
				int indX = points3.indexOf(points2.get(i));
				int indY = points3.indexOf(points2.get(j));
				Arete a = new Arete(points.get(i), points.get(j), M[indX][indY]);
				aretes.add(a);
			}
		}
		System.out.println("cpt vaut " + cpt);
		aretes.sort(Comparator.comparingDouble(Arete::poids));

		ArrayList<Arete> sol = new ArrayList<Arete>();
		int i = 0;
		while (sol.size() < points.size() - 1) {
			Arete ar = aretes.get(i++);
			int id1 = ar.getP().clusterId;
			int id2 = ar.getQ().clusterId;

			if (id1 != id2) {
				sol.add(ar);

				for (PointIdent pt : points) {
					if (pt.clusterId == id2) {
						pt.clusterId = id1;
					}
				}
			}

		}

		return sol;

	}

	public ArrayList<Arete> kruskal_with_point_density_and_budget(ArrayList<Point> points2, ArrayList<Point> points3,
			int budget, double[][] M) {

		ArrayList<Arete> aretes = new ArrayList<Arete>();

		ArrayList<PointIdent> points = new ArrayList<>();
		for (Point pt : points2) {
			points.add(new PointIdent(pt));
		}

		int cpt = 0;
		for (int i = 0; i < points.size(); i++) {
			for (int j = i + 1; j < points.size(); j++) {
				cpt++;
				int indX = points3.indexOf(points2.get(i));
				int indY = points3.indexOf(points2.get(j));
				Arete a = new Arete(points.get(i), points.get(j), M[indX][indY]);
				aretes.add(a);
			}
		}

		// check point density; points2 is hitspoints
		HashMap<Point, Double> point_density = new HashMap<>();
		for (Point p1 : points3) {
			double dist_moyenne = 0;
			for (Point p2 : points3) {
				dist_moyenne += p1.distance(p2);
			}
			point_density.put(p1, dist_moyenne / (points3.size()));
		}

		System.out.println("cpt vaut " + cpt);
		aretes.sort(Comparator.comparingDouble(Arete::poids));

		ArrayList<Arete> sol = new ArrayList<Arete>();
		int i = 0;
		int budget_used = 0;
		HashMap<PointIdent, Boolean> isInTree = new HashMap<>();

		Collections.shuffle(aretes);
		Arete root_arete = aretes.get(0);
		sol.add(root_arete);
		isInTree.put(root_arete.getP(), true);
		isInTree.put(root_arete.getQ(), true);
		budget_used += root_arete.poids();
		while (sol.size() < points.size() - 1) {

			Arete ar = null;
			double val_of_interest = Double.POSITIVE_INFINITY;
			for (Arete a : aretes) {
				if (isInTree.containsKey(a.getP()) ^ isInTree.containsKey(a.getQ())) {
					double tmp = Double.POSITIVE_INFINITY;
					if (isInTree.containsKey(a.getP())) {
						tmp = a.poids() / point_density.get(a.getP().getPoint());
					}
					if (isInTree.containsKey(a.getQ())) {
						tmp = a.poids() / point_density.get(a.getQ().getPoint());
					}

					if (val_of_interest > tmp) {
						ar = a;
						val_of_interest = tmp;
					}
				}

			}

			System.out.println("l'arete vaut " + ar);

			if (budget_used + ar.poids() > budget) {
				break;
			}

			budget_used += ar.poids();

			// Arete ar = aretes.get(i);
			int id1 = ar.getP().clusterId;
			int id2 = ar.getQ().clusterId;
			isInTree.put(ar.p, true);
			isInTree.put(ar.getQ(), true);

			if (id1 != id2) {
				sol.add(ar);

				for (PointIdent pt : points) {
					if (pt.clusterId == id2) {
						pt.clusterId = id1;
					}
				}
			}

		}

		return sol;

	}

	HashMap<Arete, Boolean> test = new HashMap<>();
	int cpt = 0;

	public Tree2D aretesToTree(Point p, ArrayList<Arete> aretes) {
		ArrayList<Arete> newArete = (ArrayList<Arete>) aretes.clone();
		ArrayList<Point> succ = new ArrayList<>();

		for (Arete a : aretes) {

			/*
			 * if(test.containsKey(a)) { System.out.println("oheee");; }
			 */
			test.put(a, true);
			// System.out.println(a);
			if (a.getP().getPoint().equals(p)) {
				succ.add(a.getQ().getPoint());
				newArete.remove(a);
			} else {
				if (a.getQ().getPoint().equals(p)) {
					succ.add(a.getP().getPoint());
					newArete.remove(a);
				}
			}
		}

		ArrayList<Tree2D> subtrees = new ArrayList<>();

		for (Point q : succ) {
			subtrees.add(aretesToTree(q, newArete));
		}
		return new Tree2D(p, subtrees);
	}

	public Tree2D areteToTree(ArrayList<Arete> aretes) {
		return aretesToTree(aretes.get(0).getP().pt, aretes);
	}

	public ArrayList<Arete> replace(Arete a, ArrayList<Point> points, int[][] paths) {
		PointIdent p = a.getP();
		PointIdent q = a.getQ();
		int nbP = -1;
		int nbQ = -1;

		for (int i = 0; i < points.size(); i++) {
			if (points.get(i).equals(p.pt)) {
				nbP = i;
			}
			if (points.get(i).equals(q.pt)) {
				nbQ = i;
			}
			if (nbP != -1 && nbQ != -1)
				break;
		}

		ArrayList<Arete> res = new ArrayList<>();

		while (nbP != nbQ) {
			Point p1 = points.get(nbP);
			Point p2 = points.get(paths[nbP][nbQ]);
			double dist = p1.distance(p2);
			res.add(new Arete(new PointIdent(p1), new PointIdent(p2), dist));
			nbP = paths[nbP][nbQ];

		}

		return res;

	}

	public Tree2D calculSteiner(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {

		ArrayList<Object> result = floydWarshall(points, edgeThreshold);

		double dist[][] = (double[][]) result.get(0);
		int paths[][] = (int[][]) result.get(1);
		ArrayList<Arete> krusk = kruskal(hitPoints, points, dist);
		ArrayList<Arete> kruskafter = new ArrayList<Arete>();
		for (Arete a : krusk) {
			kruskafter.addAll(replace(a, points, paths));
		}
		System.out.println(kruskafter.size());
		System.out.println("salut22");
		Tree2D T0 = areteToTree(kruskafter);
		System.out.println("salut22fdsfds");

		System.out.println(T0);
		return T0;
	}

	public ArrayList<Arete> reset_ident(ArrayList<Arete> aretes) {
		ArrayList<Arete> res = new ArrayList<Arete>();

		for (Arete a : aretes) {
			res.add(new Arete(new PointIdent(a.getP().getPoint()), new PointIdent(a.getQ().getPoint()), a.poids()));
		}

		return res;
	}

	public ArrayList<PointIdent> from_arete_points(ArrayList<Arete> aretes) {
		ArrayList<PointIdent> res = new ArrayList<>();

		for (Arete a : aretes) {
			res.add(a.getP());
			res.add(a.getQ());
		}
		return res;
	}

	public ArrayList<Arete> kruskal_heur(ArrayList<Arete> aretes, int budget) {
		int sum = 0;
		System.out.println("cpt vaut " + cpt);
		// Collections.shuffle(aretes);

		aretes.sort(Comparator.comparingDouble(Arete::poids));

		Arete root = aretes.get(0);
		sum += root.poids();
		ArrayList<Arete> sol = new ArrayList<Arete>();
		ArrayList<PointIdent> points = from_arete_points(aretes);
		sol.add(root);
		aretes.remove(0);
		HashMap<PointIdent, Boolean> isInSol = new HashMap<>();
		isInSol.put(root.getP(), true);
		isInSol.put(root.getQ(), true);

		System.out.println(isInSol + " isInSol");

		boolean flag = false;
		while (!flag) {
			int i = 0;
			while (i < aretes.size()) {
				Arete ar = aretes.get(i++);
				// System.out.println("ar vaut " + ar);
				if (isInSol.containsKey(ar.getP()) || isInSol.containsKey(ar.getQ())) {

					System.out.println("sum" + sum);
					if (sum + ar.poids() > budget) {
						System.out.println("sum" + sum);
						flag = true;
						break;
					}
					sum += ar.poids();
					isInSol.put(ar.getP(), true);
					isInSol.put(ar.getQ(), true);
					sol.add(ar);
					aretes.remove(ar);
					break;
				}
			}

		}

		System.out.println("budget use" + sum);
		return sol;

	}

	public Tree2D calculSteinerBudget(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
		int budget = 1664;
		ArrayList<Object> result = floydWarshall(points, edgeThreshold);

		double dist[][] = (double[][]) result.get(0);
		int paths[][] = (int[][]) result.get(1);
		ArrayList<Arete> krusk = kruskal(hitPoints, points, dist);
		// krusk = kruskal_heur(krusk, budget);
		krusk = kruskal_with_point_density_and_budget(hitPoints, points, budget, dist);
		ArrayList<Arete> kruskafter = new ArrayList<Arete>();
		for (Arete a : krusk) {
			kruskafter.addAll(replace(a, points, paths));
		}
		System.out.println(kruskafter.size());
		System.out.println("salut22");
		Tree2D T0 = areteToTree(kruskafter);
		System.out.println("salut22fdsfds");

		System.out.println(T0);

		Tree2D sol1 = T0;
		return T0;

	}
}
