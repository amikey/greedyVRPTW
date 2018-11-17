package algorithm;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import util.VrpProblem;
import util.VrpReader;
import util.VrpSolution;
import viz.VrpPanel;

public class SA {
	
	private VrpProblem problem;
	private VrpSolution initSolution;
	
	public SA(VrpProblem problem, VrpSolution initSolution) {
		super();
		this.problem = problem;
		this.initSolution = initSolution;
	}

	/**
	 * ����VrpSolution����ļ��㺯���������洢��·���� ����  �ķ�
	 * @param sol �����һ���µĿ��н⣬���������е�Ķ೵����·������
	 * @return ���س������������Ҫ��·;����ķ�
	 */
	public double toursCost(VrpSolution sol) {
		return sol.getToursCost();
	}

	/**
	 * ����·�����ϵľ��뻨��
	 * @param routes һ������·��Ϊroute�����г���·�����routes
	 * @param problem	���������
	 * @return ������Щ·�����ܻ���
	 */
	public double calcToursCost(List<List<Integer>> routes, VrpProblem problem) {
	    double[][] distances = problem.getDistances();
	    double[] distancesFromDepot = problem.getDistancesFromDepot();
	    
	    double toursCost = 0;
	    for (List<Integer> route : routes) {
	      Iterator<Integer> iter = route.iterator();
	      if (!iter.hasNext()) {
	        continue;
	      }
	      int prev = iter.next();
	      toursCost += distancesFromDepot[prev];
	      while (iter.hasNext()) {
	        int cur = iter.next();
	        toursCost += distances[prev][cur];
	        prev = cur;
	      }
	      toursCost += distancesFromDepot[prev];
	    }
	    return toursCost;
	  }
	
	public List<List<Integer>> Sa_VRPTW(VrpSolution initSolution, double T0, double d, double Tk,int L) {
		// T0=1e5,d =1-7e-3, Tk=1e-3
		VrpSolution solution = initSolution;
		List<List<Integer>> bestpath, curentpath;
		double t = T0;
		//�����ĳ�ʼ�⣬������������·����ÿһ����һ��List��������List<List<Integer>>��װ
		bestpath = copyRoutes(initSolution.getRoutes());
		curentpath = copyRoutes(initSolution.getRoutes());
		Random random = new Random();
		Operator<Integer> operator = new Operator<>();
		while (t > Tk) {
			int it=0;
			System.out.println("****** ��ǰ�¶� T="+t+"*****");
			while (it<L) {
				System.out.println("������ "+it+" ��");
				List<List<Integer>> update_path = operator.groupTwo_Exchange(curentpath);
				//�����չ��·���޷�����Լ�����������ԭʼ������չ
				while (!verify(update_path, problem)) {
//					System.out.println("-------������Լ��-------------");
					update_path = operator.groupTwo_Exchange(curentpath);
				}
				System.out.println("-------��չ��·��-------------");
				double delta = calcToursCost(update_path,problem) - calcToursCost(curentpath,problem);
				if (delta < 0) {//Ϊ��ֵ��������ɱ������ˣ������
					System.out.println("����ֵ delta = "+delta);
					curentpath = update_path;
					bestpath = update_path;
				} else {
					double p = Math.exp(-delta/t);
					if (random.nextDouble() <= p) {
						curentpath = update_path;
					}
				}
				it++;
			}
			t *=d;
		}
		return bestpath;
	}
	
	public List<List<Integer>> copyRoutes(List<List<Integer>> routes){
		List<List<Integer>> copyRoutes = new ArrayList<List<Integer>>();
		Iterator<List<Integer>> iterator = routes.iterator();
		while (iterator.hasNext()) {
			List<Integer> eachRoute = iterator.next();
			Iterator<Integer> it = eachRoute.iterator();
			List<Integer> tmpRoute = new ArrayList<Integer>();
			while (it.hasNext()) {
				tmpRoute.add(it.next());
			}
			copyRoutes.add(tmpRoute);
		}
		return copyRoutes;
	}
	
	/**
	 * �����ж����г���·���Ƿ�����Լ������
	 * @param routes
	 * @param problem
	 * @return
	 */
	public Boolean isRouteAvailable(List<List<Integer>> routes, VrpProblem problem) {
		int[] demands = problem.getDemands();
		int[] windowStartTimes = problem.getWindowStartTimes();
		int[] windowEndTimes = problem.getWindowEndTimes();
		double[][] distances = problem.getDistances();
		double[] distancesFromDepot = problem.getDistancesFromDepot();
		int[] serviceTimes = problem.getServiceTimes();
		
		for (List<Integer> route : routes) {
			int curNodeId = -1;
			double curVisitTime = 0;
			int remCapacity = problem.getVehicleCapacity();
			int curLastServiceTime = (curNodeId == -1) ? 0 : serviceTimes[curNodeId];
			for (Integer nodeId : route) {
				//�������һ��վ��ʼǰ�����ˣ�������������Լ��
				double distance = (curNodeId == -1) ? distancesFromDepot[nodeId] : distances[curNodeId][nodeId];
				//��Ϊ��һ����Ҫ����ʼʱ��֮��ŻῪʼ�����������翪ʼʱ��Ƚϣ�ѡȡ���һ��
				double minVisitTime = Math.max(curVisitTime + distance + curLastServiceTime, windowStartTimes[nodeId]);
				//����õ��Ŀ�ʼʱ����������ʼʱ�䣬���ߴﵽҪ���������޷�װ�£�������Լ��������false
				if (minVisitTime > windowEndTimes[nodeId]
						|| remCapacity<demands[nodeId]) {
					return false;
				}else {
					curNodeId = nodeId;
					curVisitTime = minVisitTime;
					remCapacity-=demands[nodeId];
				}
			}
			return true;
		}
		
		return true;
	}

	/**
	   * Verify that the solution satisfied the constraints for the given problem,
	   * and that the reported values for the objective function are correct.
	   */
	  public boolean verify(List<List<Integer>> routes, VrpProblem problem) {
	    double[][] distances = problem.getDistances();
	    double[] distancesFromDepot = problem.getDistancesFromDepot();
	    
//	    int toursCost = calcToursCost(routes, problem);
	    
	    int[] windowStartTimes = problem.getWindowStartTimes();
	    int[] windowEndTimes = problem.getWindowEndTimes();
	    int[] serviceTimes = problem.getServiceTimes();
	    int[] demands = problem.getDemands();
	    boolean[] visited = new boolean[problem.getNumCities()];
	    
	    //follow the paths and make sure that the time constraints hold
	    for (List<Integer> route : routes) {
	      Iterator<Integer> iter = route.iterator();
	      if (route.isEmpty()) {
//	        System.out.println("EMPTY ROUTE!!!");
	        continue;
	      }
	      int prev = iter.next();
	      if (windowEndTimes[prev] < distancesFromDepot[prev]) {
//	        System.out.println("first node violated time constraint: endTime=" + 
//	            windowEndTimes[prev] + ", dist=" + distancesFromDepot[prev]);
	        return false;
	      }
	      visited[prev] = true;
	      int remCapacity = problem.getVehicleCapacity() - demands[prev];
	      double minVisitTime = Math.max(windowStartTimes[prev], distancesFromDepot[prev]);
	      
	      while (iter.hasNext()) {
	        int cur = iter.next();
	        visited[cur] = true;
	        double nextMinVisitTime = Math.max(minVisitTime + serviceTimes[prev] + distances[prev][cur], windowStartTimes[cur]);
	        if (nextMinVisitTime > windowEndTimes[cur]) {
//	          System.out.println(minVisitTime + "\t" + serviceTimes[prev] + "\t" + distances[prev][cur]);
//	          System.out.println("violated time constraint for " + prev + "->" + cur + 
//	              ": endTime=" + windowEndTimes[cur] + ", visitTime=" + nextMinVisitTime);
	          return false;
	        }
	        minVisitTime = nextMinVisitTime;
	        
	        remCapacity -= demands[cur];
	        if (remCapacity < 0) {
//	          System.out.println("violated capacity constraint");
	          return false;
	        }
	        prev = cur;
	      }
	    }  
	    
	    for (boolean b : visited) {
	      if (!b) {
	        System.out.println("one of the nodes not visited");
	        return false;
	      }
	    }
	    
//	    if (toursCost != this.toursCost) {
//	      System.out.println("tour costs do not match: " + toursCost + " != " + this.toursCost);
//	      return false;
//	    }
	    
	    return true;
	  }
	
	public static void main(String[] args) throws IOException {
		File f = new File("problems/c1_2_1.txt");
	    VrpProblem problem = VrpReader.readSolomon(f, 100);
	    GreedyInitializer init = new GreedyInitializer(1.0, 1.0, 0);
	    VrpSolution sol = init.nearestNeighborHeuristic(problem);
		SA sa = new SA(problem,sol);
		double T0=1e6;
		double d=0.97;
		double Tk=1e-5;
		int L = 20*problem.getNumCities();//��ѭ������
		List<List<Integer>> bestRoutes = sa.Sa_VRPTW(sol, T0, d, Tk, L);
		
		VrpSolution bestSolution = new VrpSolution(bestRoutes, problem);
	    System.out.println(bestSolution.getNumVehicles());
	    System.out.println(bestSolution.getToursCost());
	    System.out.println(bestSolution.verify(problem));
	    
	    JFrame frame = new JFrame();
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    VrpPanel panel = new VrpPanel();
	    panel.setScale(problem);
	    panel.setSolution(bestSolution);
	    frame.getContentPane().add(panel);
	    frame.pack();
	    frame.setVisible(true);

	}

}
