package algorithms;

import java.awt.Point;
import java.util.AbstractCollection;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.Queue;


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
		for (int i = 0; i < points.size()/2 / 3; i++)
			result.add(points.get(i));
		//System.out.println(TreeToPoints(calculSteiner(points, edgeThreshold, maximalIndependentSet(points, edgeThreshold))));
		//System.out.println(IsValid(maximalIndependentSet(points, edgeThreshold), edgeThreshold ) );
		//System.out.println(maximalIndependentSet(points, edgeThreshold).size());
		//System.out.println(points.size());
		//System.out.println(IsValid(MIS2(points,edgeThreshold),edgeThreshold));
		ArrayList<Point> points4 = (ArrayList<Point>) points.clone();
		ArrayList<Point> points2 = (ArrayList<Point>) points.clone();
		//System.out.println(Verify2Hopes(MIS2(points,edgeThreshold),edgeThreshold,points2));
		//System.out.println(MIS2(points,edgeThreshold));
		//System.out.println(maximalIndependentSet(points, edgeThreshold).size());
		//return TreeToPoints(calculSteiner(points, edgeThreshold, maximalIndependentSet(points, edgeThreshold)));
		//System.out.println("voici le res : " + EDC(MIS2(points,edgeThreshold), points2, edgeThreshold).size());
		ArrayList<Point> point3 = MIS3(points, edgeThreshold);
		System.out.println("Nombre de noeuds noir" + point3.size());
		//point3.addAll(EDC(point3, points2, edgeThreshold));
		//return EDC(MIS2(points,edgeThreshold), points2, edgeThreshold);
		//System.out.println("ET VOILA " + IsValidED(points4, point3, edgeThreshold));
		//System.out.println("ET VOILA " + BFS(point3, edgeThreshold).size());
		//System.out.println("ALORS" + point3);
		//System.out.println("les voisin de lui sont" + nsOf(edgeThreshold,point3,239,642));
		//System.out.println(MIS2(points, edgeThreshold).size());
		//return MIS2(points, edgeThreshold);
		return point3;
	}
	
	public ArrayList<Point> BFS(ArrayList<Point> points, int edgeThreshold) {
		ArrayList<Point> marquee = new ArrayList<Point>();
		Queue<Integer> f = new PriorityQueue<Integer>();
		f.add(0);
		marquee.add(points.get(0));
		System.out.println("AU DEBUTTTT!!!!!!!!!!!!!" + marquee);
		while(!f.isEmpty()){
			int s = f.remove();
			System.out.println("je rentre");
			System.out.print(s);
			System.out.println(f);
			//System.out.println(neighbor(points.get(s), points, edgeThreshold));
			for (Point t : neighbor(points.get(s), points, edgeThreshold)) {
				System.out.println(marquee);
				System.out.println("CA CONTIENTtttttttttt" + marquee.contains(t));
				if(marquee.contains(t)==false) {
					System.out.println("YOOOOOOOOOOOOOOO");
					f.add(points.indexOf(t));
					//System.out.println("YOOOOOOOOOOOOOOO");
					marquee.add(t);
				}
			}
		}
		return marquee;
	}
	/*public ArrayList<Point> S-MIS(ArrayList<Point> points, int edgeThreshold){
		ArrayList<Point> blackPoints = maximalIndependentSet(points, edgeThreshold);
		ArrayList<Point> clonePointes = ((ArrayList<Point>) points.clone());
		clonePointes.removeAll(blackPoints);
		ArrayList<Point> greyPoints = clonePointes;
		ArrayList<Integer> ints = new ArrayList<Integer>();
		ints.add(5);
		ints.add(4);
		ints.add(3);
		ints.add(2);
		ArrayList<Point> bluePoints = new ArrayList<Point>();
		for(int i : ints) {
			
		}
	}*/
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
	/*public boolean Verify2Hopes(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> fP) {
		System.out.println("FP"+ fP);
		for(Point p : points) {
			for(Point q : points) {
				if (p!=q && !voisDevois(p, fP, edgeThreshold).contains(q)) {
					System.out.println(p);
					System.out.println(q);
					System.out.println(voisDevois(p, fP, edgeThreshold));
					return false;
				}
			}
		}
		return true;
	}*/
	public ArrayList<Point> nsOf(int edgeThreshold,ArrayList<Point> points , int x , int y){
		for(Point p : points) {
			if ((p.x == x) && (p.y == y)){
				return neighbor(p, points, edgeThreshold);
			}
		}
		return null;
	}
	
	public ArrayList<Point> voisDevois(Point p, ArrayList<Point> points,ArrayList<Point> b,int edgeThreshold){
		ArrayList<Point> results = new ArrayList<Point>();
		for(Point r : neighbor(p, points, edgeThreshold)) {
			for (Point q : neighbor(r, points, edgeThreshold)) {
				if (q!=null && b.contains(q)) {
					results.add(q);
				}
			}
		}
		return results;
		/*if(results.get(0)!=null) {
			return results.get(0);
		}
		else {
			return null;
		}
		/*for (Point s : results) {
			for(Point r : neighbor(s, points, edgeThreshold)) {
				for (Point q : neighbor(r, points, edgeThreshold)) {
					if (b.contains(q)) {
						return q;
					}
				}
			}
		}
		return null;*/
	}
	
	public HashMap<Point, Integer> voisinDevoisin(ArrayList<Point> points,HashMap<Point,Integer> degrees,Point p,int edgeThreshold,ArrayList<Point> w){
		ArrayList<Point> results = new ArrayList<Point>();
		for(Point r : neighbor(p, points, edgeThreshold)) {
			for (Point q : neighbor(r, points, edgeThreshold)) {
				if (q!=null && w.contains(q)) {
					int val = degrees.get(q);
					if(val!=0)
						val = val - 1;
						degrees.put(q,val);
				}
			}
		}
		return degrees;
	}
	
	public Point min(HashMap<Point, Integer> degrees,ArrayList<Point> b) {
		System.out.println(b);
		Point min = b.get(0);
		for(Point p : degrees.keySet()) {
			if(b.contains(p) && degrees.get(p)<degrees.get(min)) {
				min = p;
			}
		}
		return min;
	}
	
	public ArrayList<Point> MIS2 (ArrayList<Point> points, int edgeThreshold){
		ArrayList<Point> whitePs = points;
		ArrayList<Point> blackPs = new ArrayList<Point>();
		ArrayList<Point> greyPs = new ArrayList<Point>();
		HashMap<Point, Integer> degrees = new HashMap<Point, Integer>();
		
		for(Point p : points) {
			ArrayList<Point> ne = neighbor(p, points, edgeThreshold);
			if (ne != null) {
				degrees.put(p,ne.size());
			}else {
				degrees.put(p,0);
			}
		}
		
		Point start = whitePs.get(0);
		while (whitePs.size() > 0) {
			whitePs.remove(start);
			blackPs.add(start);
			greyPs.addAll(neighbor(start, points, edgeThreshold));
			whitePs.removeAll(neighbor(start, points, edgeThreshold));
			degrees = voisinDevoisin(points, degrees, start, edgeThreshold,whitePs);
			if(whitePs.size()>0)
				start = min(degrees,whitePs);
		}
		System.out.println("GRIS EST" + greyPs);
		return blackPs;
	}
	public Point hasVoisDeVoisBlack(Point p, ArrayList<Point> points,int edgeThreshold,ArrayList<Point> black) {
		for (Point s : voisDevois(p, points, black, edgeThreshold)){
			return s;
		}
		return null;
		
	}
	
	public ArrayList<Point> MIS3 (ArrayList<Point> points, int edgeThreshold){
		ArrayList<Point> whitePs = points;
		ArrayList<Point> blackPs = new ArrayList<Point>();
		ArrayList<Point> greyPs = new ArrayList<Point>();
		
		
		Point start = whitePs.get(0);
		System.out.println("je suis premier" + whitePs.get(0));
		while (whitePs.size() > 0) {
			System.out.println("whar");
			System.out.println(whitePs);
			whitePs.remove(start);
			blackPs.add(start);
			greyPs.addAll(neighbor(start, points, edgeThreshold));
			whitePs.removeAll(neighbor(start, points, edgeThreshold));
			if (whitePs.size()>0) {
				
				/*for(Point p : whitePs) {
					for(Point s : neighbor(p, points, edgeThreshold)) {
						for(Point q : neighbor(s, points, edgeThreshold)) {
							System.out.println("JE COMPREND PAS" + neighbor(q, points, edgeThreshold));
							if(blackPs.contains(q)) {
								System.out.println("je rentre pas la");
								start = p;
							}
							else {
								start = whitePs.get(0);

							}
						}
					}
				}*/
				start = whitePs.get(0);

			}
			//start = whitePs.get(0);

		}

		System.out.println("GRIS EST" + greyPs);
		return blackPs;
				//start = whitePs.get(0);
				//hasVoisDeVoisBlack(p, points, edgeThreshold, blackPs);
		}
	
	//Cette fonction renvoie la liste des composants aux quels p apprtient
	public ArrayList<ArrayList<Point>> ListOfCompos(ArrayList<ArrayList<Point>> comps,Point p ){
		ArrayList<ArrayList<Point>> res = new ArrayList<ArrayList<Point>>();
		for(ArrayList<Point> a : comps) {
			if(a.contains(p)) {
				res.add(a);
			}
		}
		return res;
	}
	public boolean hasBlack(ArrayList<ArrayList<Point>> comp,ArrayList<Point> black) {
		for(ArrayList<Point> l : comp)
		{
			for(Point p : l) {
				if (black.contains(p)){
					return true;
				}
			}
		}
		return false;
	}
	
	public boolean IsAdjInToIComposent(Point p,ArrayList<Point> points,ArrayList<ArrayList<Point>> comps,int edgeThreshold,int nbr,ArrayList<Point> blackPs) {
		ArrayList<ArrayList<Point>> visited = new ArrayList<ArrayList<Point>>();
		
		if(p.x == 239 && p.y == 642) {
			System.out.println("BIEN SUR");
		}
		for(Point v : neighbor(p, points, edgeThreshold)){
			if(blackPs.contains(v)) {
				//System.out.println("j'arrive quand meme a venir la");
				ArrayList<ArrayList<Point>> list = ListOfCompos(comps,v);
				//if(hasBlack(list, blackPs)) {
				//java.awt.Point[x=296,y=635], java.awt.Point[x=317,y=620]
				if(p.x == 265 && p.y == 625) {
					System.out.println("KHODAYA" + v + " " + list.size());
				}
				if(list!=null) {
					for(ArrayList<Point> a : list) {
						if (!visited.contains(a)) {
							visited.add(a);
						}
					}
				}
			//}
			}
		}
		//le noeud gris qui devient pas bleu
		if(p.x == 265 && p.y == 625) {
			System.out.println("VISITED" + visited);
			System.out.println("VOISIIIINNN" + neighbor(p, points, edgeThreshold));
		}
		
		return visited.size()>=nbr;
	}
	
	public Point existsGreyNodeThat(ArrayList<Point> greyPs,ArrayList<Point> points,ArrayList<ArrayList<Point>> comps,int edgeThreshold,int nbr,ArrayList<Point> blackPs) {
		for(Point p : greyPs) {
			if (IsAdjInToIComposent(p, points, comps, edgeThreshold, nbr, blackPs)) {
				return p;
			}
		}
		
		return null;
	}
	
	public ArrayList<ArrayList<Point>> unionOfCompos(ArrayList<ArrayList<Point>> components,Point p,ArrayList<Point> points,int edgeThreshold,ArrayList<Point> blacks){
		
		ArrayList<Point> union = new ArrayList<>();
		for(Point v : neighbor(p, points, edgeThreshold)) {
			if(blacks.contains(v)) {
				ArrayList<ArrayList<Point>> list = ListOfCompos(components,v);
				
				
				if(list.size()>0) {
					//System.out.println("!!!!!!!!!!!!!!!" + v + list.get(0));
					union.addAll(list.get(0)); //On cree la nouvelle composant
					//System.out.println("??????U" + union);
					components.remove(list.get(0)); //On enelve les anciennes composantes
				}
			}
		}
		
		union.add(p);
		components.add(union);
		System.out.println("nouveau compos" + components);
		return components;
	}
	
	public ArrayList<Point> EDC(ArrayList<Point> MIS,ArrayList<Point> points,int edgeThreshold){
		ArrayList<ArrayList<Point>> components = new ArrayList<ArrayList<Point>>();
		for(Point p : MIS) {
			ArrayList<Point> a = new ArrayList<Point>();
			a.add(p);
			components.add(a);
		}
		ArrayList<Point> blackPs = MIS;
		for(Point v : blackPs) {
			if(v.x == 239 && v.y == 642) {
				System.out.println("BIEN SURRRRRRRRRRRRRRRRRR" + neighbor(v, points, edgeThreshold));
			}
		}
		ArrayList<Point> bluePs = new ArrayList<Point>();
		ArrayList<Point> greyPs = (ArrayList<Point>) points.clone();
		greyPs.removeAll((Collection<Point>) MIS.clone());
		System.out.println("GRISGRIIIIS" + greyPs.size());
		ArrayList<Integer> nums = new ArrayList<Integer>();
		nums.add(5);
		nums.add(4);
		nums.add(3);
		nums.add(2);
		
		System.out.println("je suis laaaa" + blackPs);
		System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		for(int i : nums) {
			while(existsGreyNodeThat(greyPs, points, components, edgeThreshold, i, blackPs)!=null) {
					Point p=existsGreyNodeThat(greyPs, points, components, edgeThreshold, i, blackPs);
					System.out.println("P" + p);
					greyPs.remove(p);
					bluePs.add(p);
					System.out.println(i+ "!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					System.out.println(bluePs);
					components = unionOfCompos(components, p, points, edgeThreshold, blackPs);
				}
			System.out.println("greyPs : mmmmmmmm" + greyPs);
			}
		System.out.println(greyPs);
		System.out.println("blueeeeeeeeeeeeeeeeeeeeeeeeees" + bluePs);
		return bluePs;
	}
	
	public boolean IsValid(ArrayList<Point> vertices, int edgeThreshold ) {	
		for(Point p : vertices) {
			if(neighbor(p, vertices, edgeThreshold).size() > 0) {
				return false;
			}
		}
		return true;
		
	}
	private boolean IsValidED(ArrayList<Point> vertices, ArrayList<Point> DS, int edgeThreshold) {
		for (Point p : DS) {
			vertices.remove(p);
			vertices.removeAll(neighbor(p, vertices, edgeThreshold));
		}
		Boolean isv = vertices.size() == 0;
		return vertices.size() == 0;
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
		Tree2D T0 = areteToTree(kruskafter);
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
