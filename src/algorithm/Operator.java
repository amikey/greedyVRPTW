package algorithm;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

/**
 * ����������֪��󣬲��������½������еĸ��ֲ�������
 * ����Two-Exchange��Or-Exchange��Relocation��Exchange��Crossover���� ��Դ��A Two-Stage
 * Hybrid Local Search for the Vehicle Routing Problem with Time
 * Windows�����õ�����Щ���� �ƻ���������ӣ�2-opt��k-opt������
 * 
 * @author Intgrp
 *
 */
public class Operator<T> {

	/**
	 * �Ե�ǰ·����i,j֮��ĵ���з�ת������i��j��������
	 * 
	 * @param a
	 * @param i
	 * @param j
	 * @return
	 */
	public List<T> two_Exchange(List<T> a, int i, int j) {
		// ��i��j�����˵㣬����ĵ�����ת
		for (int m = i; m <= (j - i) / 2 + i; m++) {
			T tmp = a.get(m);
			a.set(m, a.get(j - m + i));
			a.set(j - m + i, tmp);
		}
		return a;
	}

	public List<List<T>> groupTwo_Exchange(List<List<T>> routes) {
		// Ҫ����һ���µĲ�������Ҫ�ȶ�ԭʼ·������������ֹ������ó��Ľ����ԭ������������б���ԭ����·�����Լ�������
		List<List<T>> copyRoutes = new ArrayList<List<T>>();
		Iterator<List<T>> iterator = routes.iterator();
		while (iterator.hasNext()) {
			List<T> eachRoute = iterator.next();
			Iterator<T> it = eachRoute.iterator();
			List<T> tmpRoute = new ArrayList<T>();
			while (it.hasNext()) {
				tmpRoute.add(it.next());
			}
			copyRoutes.add(tmpRoute);
		}
		
		
		
		// ���ѡ�����Ҫ���������ӵ���һ��·��
		Random random = new Random();
		int routeIndex = random.nextInt(copyRoutes.size());
		// �����Ҫ����������·��
		List<T> r1 = copyRoutes.get(routeIndex);
		while (r1.size()<=1) {
			routeIndex = random.nextInt(copyRoutes.size());
			// �����Ҫ����������·��
			r1 = copyRoutes.get(routeIndex);
		}
		
		int idx1 = random.nextInt(r1.size());
		int idx2 = random.nextInt(r1.size());
//		System.out.println("i="+idx1+"��\t j="+idx2);
		if (idx1>idx2) {
			int tmp = idx1;
			idx1 = idx2;
			idx2 =tmp;
		}
		while (idx2-idx1<=1 || idx2==idx1) {
			idx1 = random.nextInt(r1.size());
			idx2 = random.nextInt(r1.size());
//			System.out.println("r1 size="+r1.size());
//			System.out.println("i="+idx1+"��\t j="+idx2);
		}
//		System.out.println("r1="+r1);
		r1 = two_Exchange(r1, idx1,idx2);
		copyRoutes.set(routeIndex, r1);
//		System.out.println("r1="+r1);
		return copyRoutes;
	}

	/**
	 * ��a·����ɾ��1��2��3�������㣬Ȼ���ƶ���b·���У�����ֻ������a,b�н϶̳��ȵķ�Χ��������ȥ
	 * 
	 * @param a
	 *            ���飨Ⱦɫ�壩a
	 * @param b
	 *            ���飨Ⱦɫ�壩b
	 * @return
	 */
	public List<List<T>> or_Exchange(T[] a, T[] b) {
		Random random = new Random();
		int moveNum = random.nextInt(3) + 1;// Ҫɾ���ƶ�������
		int index = random.nextInt((Math.min(a.length, b.length))) + 1;// ���ĸ�λ�ÿ�ʼ
		// ���λ�ú����������moveNum�������������
		while (index + moveNum >= a.length) {
			index = random.nextInt() % (Math.min(a.length, b.length));
		}
		System.out.println("index=" + index);
		System.out.println("moveNum=" + moveNum);
		// ��a�У�ɾ��index����moveNum��Ԫ��
		List<List<T>> out = new ArrayList<>();
		List<T> tmp = new ArrayList<>();
		for (int i = 0; i < a.length; i++) {
			if (i >= index && i < index + moveNum)
				continue;
			else
				tmp.add(a[i]);
		}
		out.add(tmp);
		tmp = new ArrayList<>();
		for (int i = 0; i < index + moveNum; i++) {
			if (i >= index && i < index + moveNum)
				tmp.add(a[i]);
			else
				tmp.add(b[i]);
		}
		for (int i = index; i < b.length; i++)
			tmp.add(b[i]);
		out.add(tmp);
		return out;
	}

	/**
	 * ��a·����ɾ��1��2��3�������㣬Ȼ���ƶ���b·���У�����ֻ������a,b�н϶̳��ȵķ�Χ��������ȥ
	 * 
	 * @param routes
	 *            ���г�������·����ɵļ���
	 * @return ����һ���µ�routes�����������·�������˺���or_Exchange��Ч��
	 */
	public List<List<T>> or_Exchange(List<List<T>> routes) {
		// Ҫ����һ���µĲ�������Ҫ�ȶ�ԭʼ·������������ֹ������ó��Ľ����ԭ������������б���ԭ����·�����Լ�������
		List<List<T>> copyRoutes = new ArrayList<List<T>>(routes);
		System.out.println("copyRoutes size=" + copyRoutes.size());
		// ���ѡ�����Ҫ���������ӵ�������·��
		Random random = new Random();
		int idx1 = random.nextInt(copyRoutes.size());
		int idx2 = random.nextInt(copyRoutes.size());
		while (idx1 == idx2) {
			idx2 = random.nextInt(copyRoutes.size());
		}
		System.out.println("idx1 = " + idx1 + "��\t" + "idx2=" + idx2);
		// �����Ҫ������������·��
		List<T> r1 = copyRoutes.get(idx1);
		List<T> r2 = copyRoutes.get(idx2);
		System.out.println("r1 size =" + r1.size() + "��\t" + "r2 size=" + r2.size());
		// �������Ҫɾ����������λ�ã�Ȼ����ܹ�����һ��·����ɾ����������һ��·����
		int moveNum = random.nextInt(3) + 1;// Ҫɾ���ƶ�������
		// �������·��ֻ��2-3���㣬��ʱ��Ҫ��ȥ����Ŀ�ͻᳬ�����е���������ʱ��Ͳ��ƶ��ˣ�ֱ�ӷ���ԭʼ������
		int index = random.nextInt((Math.min(r1.size(), r2.size()))) + 1;// ���ĸ�λ�ÿ�ʼ
		System.out.println("ǰmoveNum=" + moveNum + "��\t" + "index=" + index);
		// ���λ�ú����������moveNum�������������
		if ((index + moveNum >= r1.size()) || (index + moveNum >= r2.size())) {
			index = 0;
			moveNum = 1;
		}
		System.out.println("��moveNum=" + moveNum + ",\t" + "index=" + index);
		// ��a�У�ɾ��index����moveNum��Ԫ�أ�����¼����ЩԪ����Ϣ��������뵽r2��indexλ��
		List<T> tmp = new ArrayList<T>();
		for (int i = index; i < index + moveNum; i++) {
			tmp.add(r1.get(i));
		}
		// �Ƴ�ԭʼ������indexλ��֮���moveNum������
		int cnt = moveNum;
		while (cnt > 0) {
			r1.remove(index);
			cnt--;
		}
		r2.addAll(index, tmp);
		return copyRoutes;

	}

	/**
	 * a���飨Ⱦɫ�壩����������е������㣬�ڱ�����������ֱ�ӽ�����������������
	 * ������Ե���List�ģ�����List<List<T>>��������������һ�����������������ʵ��
	 * 
	 * @param a
	 *            ���飨Ⱦɫ��
	 */
	public List<T> Swap(List<T> a) {
		Random random = new Random();
		int r1 = random.nextInt(a.size());
		int r2 = random.nextInt(a.size());
		while (r1 == r2) {
			r2 = random.nextInt(a.size());
		}
		T tmp = a.get(r1);
		a.set(r1, a.get(r2));
		a.set(r2, tmp);
		return a;
	}

	public static void print(Object[] o) {
		for (int i = 0; i < o.length; i++) {
			System.out.print(o[i].toString() + " ");
		}
		System.out.println();
	}

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Integer[] num1 = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
		Integer[] num2 = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
		Operator<Integer> opera = new Operator<Integer>();
		// ����two_Exchange
		// Integer[] out = opera.two_Exchange(num1, 2, 8);
		// print(out);
		// ����or_Exchange
		// List<List<Integer>> mList ;
		// mList = opera.or_Exchange(num1, num2);
		// for (List<Integer> ml:mList) {
		// for (Integer tmp:ml) {
		// System.out.print(tmp+" ");
		// }
		// System.out.println();
		// }
		// ����Swap
		// opera.Swap(num1);
		// for (Integer t:num1) {
		// System.out.print(t+" ");
		// }
		// System.out.println();
	}

}
