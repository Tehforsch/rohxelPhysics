package physics.collision;
import java.io.Serializable;
import java.util.Vector;

import physics.Body;
import physics.PhysicalConstants;
import physics.World;
import physics.shapes.Circle;
import physics.shapes.Polygon;
import tools.Log;
/**
 * @author toni Handles collision detection. Manages broad-, mid- and narrowphase Consistently keeps track of body-pairs that are flagged for collision checks. This is accomplished by a sweep-and-prune algorithm For every pair in the resulting list, mid- and narrowphase calculations are performed. The resulting data is then stored in the pair class itself, being the point, normal and depth of collision. The contact - class is held in the pair class as well. Each call of handle will return a list of contacts that can be accessed directly by the contact solver Contacts must not be recreated every frame but instead kept alive and then get updated in each frame. Current solution: Keep pairs stored in quadratic array (do not access directly, will be changed in future versions) so that the contact pair between body i and j is pairs[i][j] = pairs[j][i]
 * 
 */
public class CollisionHandler implements Serializable {
	public static final int RESERVEDSPACE = 300;
	private World world;
	private Pair[][] pairs;
	private Vector<Pair> pairsList;
	private Broadphase broadphase;
	private int numBodies;
	public CollisionHandler(World w) {
		world = w;
		numBodies = world.numBodies;
		pairs = new Pair[numBodies + RESERVEDSPACE][numBodies + RESERVEDSPACE];
		pairsList = new Vector<Pair>();
		broadphase = new Broadphase(world, PhysicalConstants.DT, pairs);
	}
	public Vector<Pair> handle(float dt) {
		Vector<Pair> contacts = new Vector<Pair>();
		broadphase.update(dt, pairsList);
		for (Pair p : pairsList) {
			ColInfo c = null;
			if (ignoreCollision(p)) {
				continue;
			}
			if (p.b1.shape instanceof Polygon) {
				if (p.b2.shape instanceof Polygon) {
					c = doPolygonPolygon(dt, p);
				}
				if (p.b2.shape instanceof Circle) {
					c = doPolygonCircle(dt, p);
				}
			}
			if (p.b1.shape instanceof Circle) {
				if (p.b2.shape instanceof Polygon) {
					c = doCirclePolygon(dt, p);
				}
				if (p.b2.shape instanceof Circle) {
					c = doCircleCircle(dt, p);
				}
			}
			if (c != null) {
				p.update(c);
				contacts.add(p);
			}
		}
		return contacts;
	}
	/**
	 * Determines whether a collision between a pair of bodies should be processed or not. Collisions should not be processed if: 1. Both bodies are static. 2. Both bodies are asleep. The following two requirements make it possible to exclude some bodies from collision with certain other bodies because: If two bodies are in the same collision body and not in the default collision body -1 or in the default collision group -1 and they are in different collision groups then they will not collide.
	 * 
	 * @param p
	 * @return
	 */
	private boolean ignoreCollision(Pair p) {
		if (p.b1.isStatic() && p.b2.isStatic())
			return true;
		if (p.b1.sleeps && p.b2.sleeps)
			return true;
		return !p.b1.collidesWith(p.b2);
	}
	private ColInfo doPolygonPolygon(float dt, Pair p) {
		return PolygonPolygon.collision(p.b1, p.b2, (Polygon) p.b1.shape, (Polygon) p.b2.shape, dt);
	}
	private ColInfo doCircleCircle(float dt, Pair p) {
		return CircleCircle.collision(p.b1, p.b2, (Circle) p.b1.shape, (Circle) p.b2.shape, dt);
	}
	private ColInfo doCirclePolygon(float dt, Pair p) {
		ColInfo c = PolygonCircle.collision(p.b2, p.b1, (Polygon) p.b2.shape, (Circle) p.b1.shape, dt);
		if (c != null)
			c.normal.sNeg();
		return c;
	}
	private ColInfo doPolygonCircle(float dt, Pair p) {
		ColInfo c = PolygonCircle.collision(p.b1, p.b2, (Polygon) p.b1.shape, (Circle) p.b2.shape, dt);
		return c;
	}
	public void addBody(Body b) {
		numBodies++;
		if (numBodies > pairs.length) {
			extendArray();
		}
		broadphase.addBody(b);
	}
	private void extendArray() {
		Log.p("erweitere array in CollisionHandler.java");
		Pair[][] newPairs = new Pair[numBodies + RESERVEDSPACE][numBodies + RESERVEDSPACE];
		for (int i = 0; i < pairs.length; i++) {
			for (int j = 0; j < pairs.length; j++) {
				newPairs[i][j] = pairs[i][j];
			}
		}
		pairs = newPairs;
		broadphase.setPairArray(pairs);
	}
	public Vector<Pair> remBody(Body b) {
		numBodies--;
		// Save every pair that is connected to this body.
		// Return this array, so that the world outside can do something with
		// it.
		Vector<Pair> allPairsWithThisBody = getAllPairs(b);
		// Resort array correctly
		// [0 0 0]
		// [0 0 1]
		// [0 1 0]
		// becomes
		// [0 1 0]
		// [1 0 0]
		// [0 0 0]
		// if the first body is removed
		// (0 represents null, 1 represents a pair)
		Pair p;
		for (int i = 0; i < pairs.length; i++) {
			p = pairs[i][b.indexInWorld];
			if (p != null) {
				if (i < b.indexInWorld) { // The pair is always in the body with
											// the lower id
					p.otherThan(b).pairs.remove(p);
				}
				else {
					b.pairs.remove(p);
				}
				pairs[i][b.indexInWorld] = null;
				pairs[b.indexInWorld][i] = null;
			}
		}
		for (int i = 0; i < pairs.length; i++) {
			for (int j = i + 1; j < pairs.length; j++) {
				if (pairs[i][j] != null) {
					int realIdI = pairs[i][j].b1.indexInWorld;
					int realIdJ = pairs[i][j].b2.indexInWorld;
					// Delete old array entry and swap it with the new, correct
					// position
					// Assumes that the pair at [realIdI][realIdJ] has already
					// been moved to its correct place.
					// This should always be true
					Pair tmp = pairs[i][j];
					pairs[i][j] = null;
					pairs[j][i] = null;
					pairs[realIdI][realIdJ] = tmp;
					pairs[realIdJ][realIdI] = tmp;
				}
			}
		}
		broadphase.remBody(b);
		return allPairsWithThisBody;
	}
	public Vector<Pair> getAllPairs(Body body) {
		Vector<Pair> pairsWithThisBody = new Vector<Pair>();
		for (int i = 0; i < pairs.length; i++) {
			if (pairs[i][body.indexInWorld] != null) {
				pairsWithThisBody.add(pairs[i][body.indexInWorld]);
			}
		}
		return pairsWithThisBody;
	}
	public void setUpPairs(Body b1) {
		b1.pairs = new Vector<Pair>();
		for (int i = b1.indexInWorld; i < numBodies; i++) {
			Pair p = pairs[b1.indexInWorld][i];
			if (p != null)
				b1.pairs.add(p);
		}
	}
}
