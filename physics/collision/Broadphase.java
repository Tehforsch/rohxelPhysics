package physics.collision;
import java.io.Serializable;
import java.util.Vector;

import physics.Body;
import physics.World;
/**
 * Implements a sweep and prune algorithm to determine possible colliding body-pairs. To accomplish this, two lists of "delimiters", data structures that keep track of a single coordinate of one boundary of a body are stored. They are sorted each frame and whenever two delimiters start overlapping, a pair is added, whenenver two delimiters stop overlapping the pair is removed. Pairs are only added if delimiters overlap on both axes. A quick box check (with the delimiter indices) is done to achieve this.
 * 
 * @author toni
 * 
 */
public class Broadphase implements Serializable {
	private World world;
	private Delimiter[] delX;
	public Delimiter[] delY;
	private int numBodies;
	public Pair[][] pairArray;
	/**
	 * Initializes the broadphase. Most of the times there won't be any bodies in the World at this point of time.
	 * 
	 * @param w
	 * @param dt
	 * @param pairs
	 */
	public Broadphase(World w, float dt, Pair[][] pairs) {
		world = w;
		numBodies = 0;
		pairArray = pairs;
		delX = new Delimiter[2 * (numBodies + CollisionHandler.RESERVEDSPACE)];
		delY = new Delimiter[2 * (numBodies + CollisionHandler.RESERVEDSPACE)];
		Body b;
		for (int i = 0; i < world.numBodies; i++) {
			b = world.getBody(i);
			addBody(b);
		}
	}
	// public Broadphase(World w, float dt, Pair[][] pairs) {
	// world = w;
	// numBodies = world.numBodies;
	// pairArray = pairs;
	// delX = new Delimiter[2 * (numBodies + CollisionHandler.RESERVEDSPACE)];
	// delY = new Delimiter[2 * (numBodies + CollisionHandler.RESERVEDSPACE)];
	// for (int i = 0; i < numBodies; i++) {
	// Body b = world.getBody(i);
	// float vX = Math.abs(b.vel.x);
	// float vY = Math.abs(b.vel.y);
	// delX[2 * i] = new Delimiter(b, b.pos.x - b.width - vX * dt -
	// World.BOUNDINGMARGIN, true);
	// delX[2 * i + 1] = new Delimiter(b, b.pos.x + b.width + vX * dt +
	// World.BOUNDINGMARGIN, false);
	// delY[2 * i] = new Delimiter(b, b.pos.y - b.height - vY * dt -
	// World.BOUNDINGMARGIN, true);
	// delY[2 * i + 1] = new Delimiter(b, b.pos.y + b.height + vY * dt +
	// World.BOUNDINGMARGIN, false);
	// }
	// sort();
	// }
	/**
	 * Updates the delimiter positions for every body. Does not! sort the delimiters or find out new pairs / old ones that need to be removed.
	 * 
	 * @param dt
	 */
	public void refresh(float dt) {
		for (int i = 0; i < numBodies * 2; i++) {
			Delimiter dX = delX[i];
			Delimiter dY = delY[i];
			float vX = Math.abs(dX.body.vel.x);
			float vY = Math.abs(dY.body.vel.y);
			if (dX.isStart)
				dX.pos = dX.body.pos.x - dX.body.width - vX * dt - World.BOUNDINGMARGIN;
			else
				dX.pos = dX.body.pos.x + dX.body.width + vX * dt + World.BOUNDINGMARGIN;
			if (dY.isStart)
				dY.pos = dY.body.pos.y - dY.body.height - vY * dt - World.BOUNDINGMARGIN;
			else
				dY.pos = dY.body.pos.y + dY.body.height + vY * dt + World.BOUNDINGMARGIN;
		}
	}
	/**
	 * Theoretically this should work as follows: Every delimiter is updated with the current position of its body. The delimiters get sorted along the x- and y-axis. Whenenver two delimiters start to overlap because of the sorting along one axis, addPairX or addPairY is called. addPairX and addPairY only really add a new pair if the other axis overlaps aswell. TODO : Does this actually help against just checking for overlap on one axis?
	 * 
	 * @param dt
	 * @param pairs
	 * @param pairsList
	 */
	public void update(float dt, Vector<Pair> pairsList) {
		refresh(dt);
		updateDelimiters(delX, true);
		updateDelIndicesX();
		updateDelimiters(delY, false);
		updateDelIndicesY();
		// Collect all pairs in pairsList
		pairsList.clear();
		for (int i = 0; i < numBodies; i++) {
			Body b = world.getBody(i);
			for (Pair p : b.pairs) {
				pairsList.add(p);
			}
		}
	}
	private void updateDelIndicesX() {
		for (int i = 0; i < numBodies * 2; i++) {
			if (delX[i].isStart)
				delX[i].body.delIdStartX = i;
			else
				delX[i].body.delIdEndX = i;
		}
	}
	private void updateDelIndicesY() {
		for (int i = 0; i < numBodies * 2; i++) {
			if (delY[i].isStart)
				delY[i].body.delIdStartY = i;
			else
				delY[i].body.delIdEndY = i;
		}
	}
	/**
	 * Basically performs insertion sort on a given array of delimiters. Whenenver it moves two delimiters along eachother (See the implementation of insertion sort as an explanation) it will add/remove the collision pair of those two bodies.(depending on what kind of delimiters pass each other (start <-> start, start <-> end, ...)
	 * 
	 * @param dels
	 *            The array of delimiters. Will only be delX, or delY but this function is written for the general case because it has to do the same things.
	 * @param isX
	 *            Determines whether the array of delimiters is delX or delY.
	 * @return
	 */
	private Delimiter[] updateDelimiters(Delimiter[] dels, boolean isX) {
		float keyPos;
		Delimiter key, passingDel;
		int i;
		for (int j = 1; j < 2 * numBodies; j++) {
			key = dels[j];
			keyPos = key.pos;
			i = j - 1;
			while (i > 0 && dels[i].pos > keyPos) { // As long as the delimiter
													// key is not on the correct
													// position
				passingDel = dels[i];
				if (key.isStart && !passingDel.isStart) { // Start passes end
					if (isX) {
						addPairX(key.body, passingDel.body);
					}
					else {
						addPairY(key.body, passingDel.body);
					}
				}
				else if (!key.isStart && passingDel.isStart) {
					remPair(key.body, passingDel.body);
				}
				dels[i + 1] = passingDel;
				i--;
			}
			dels[i + 1] = key;
		}
		// Make sure the first two elements are sorted as well (fails sometimes)
		if (numBodies != 0 && dels[0].pos > dels[1].pos) {
			key = dels[0];
			dels[0] = dels[1];
			dels[1] = key;
		}
		return dels;
	}
	private void addPairX(Body b1, Body b2) {
		if (b1.delIdStartY > b2.delIdEndY || b2.delIdStartY > b1.delIdEndY) {
			return; // They dont overlap along the y-axis
		}
		addPair(b1, b2);
	}
	private void addPairY(Body b1, Body b2) {
		if (b1.delIdStartX > b2.delIdEndX || b2.delIdStartX > b1.delIdEndX) {
			return; // They dont overlap along the x-axis
		}
		addPair(b1, b2);
	}
	private void addPair(Body b1, Body b2) {
		if (pairArray[b1.indexInWorld][b2.indexInWorld] == null) { // No pair
																	// between
																	// these
																	// bodies
																	// existed
																	// so far
			Pair p;
			for (Pair p2 : b1.pairs)
				assert (p2.otherThan(b1) != b2);
			for (Pair p2 : b2.pairs)
				assert (p2.otherThan(b2) != b1);
			if (b1.indexInWorld < b2.indexInWorld) {
				p = new Pair(b1, b2);
				b1.pairs.add(p);
			}
			else {
				p = new Pair(b2, b1);
				b2.pairs.add(p);
			}
			pairArray[b1.indexInWorld][b2.indexInWorld] = p;
			pairArray[b2.indexInWorld][b1.indexInWorld] = p;
		}
	}
	private void remPair(Body b1, Body b2) {
		pairArray[b1.indexInWorld][b2.indexInWorld] = null;
		pairArray[b2.indexInWorld][b1.indexInWorld] = null;
		if (b1.indexInWorld < b2.indexInWorld)
			remPairFromBody(b1, b2);
		else
			remPairFromBody(b2, b1);
	}
	private void remPairFromBody(Body toRemoveFrom, Body toBeRemoved) {
		Pair correctPair = null;
		for (Pair p : toRemoveFrom.pairs) {
			if (p.otherThan(toRemoveFrom) == toBeRemoved) {
				correctPair = p;
				break;
			}
		}
		if (correctPair != null)
			toRemoveFrom.pairs.remove(correctPair);
		// if correctPair is null then the system just tries to remove a pair
		// that has not been added because it didn't overlap in x direction
		// anyway.
	}
	public void addBody(Body b) {
		numBodies++;
		if (2 * numBodies >= delX.length) {
			extendArrays();
		}
		assert (delX[2 * numBodies] == null);
		assert (delX[2 * numBodies + 1] == null);
		assert (delY[2 * numBodies] == null);
		assert (delY[2 * numBodies + 1] == null);
		delX[2 * numBodies - 2] = new Delimiter(b, b.pos.x - b.width - World.BOUNDINGMARGIN, true);
		delX[2 * numBodies - 1] = new Delimiter(b, b.pos.x + b.width + World.BOUNDINGMARGIN, false);
		delY[2 * numBodies - 2] = new Delimiter(b, b.pos.y - b.height - World.BOUNDINGMARGIN, true);
		delY[2 * numBodies - 1] = new Delimiter(b, b.pos.y + b.height + World.BOUNDINGMARGIN, false);
		sort();
		updateDelIndicesX();
		updateDelIndicesY();
		findPairsForNewBody(b);
	}
	private void extendArrays() {
		Delimiter[] newDelX, newDelY;
		newDelX = new Delimiter[delX.length + CollisionHandler.RESERVEDSPACE * 2];
		newDelY = new Delimiter[delY.length + CollisionHandler.RESERVEDSPACE * 2];
		for (int i = 0; i < delX.length; i++) {
			newDelX[i] = delX[i];
			newDelY[i] = delY[i];
		}
		delX = newDelX;
		delY = newDelY;
	}
	public void remBody(Body b) {
		// Delete delimiters out of delimiter array.
		cleanDelimiter(delX, b, false);
		cleanDelimiter(delX, b, true);
		cleanDelimiter(delY, b, false);
		cleanDelimiter(delY, b, true);
		numBodies--;
	}
	/**
	 * Is called whenever a body is removed. Will set the array position that contains the bodies delimiter to null and shift all following array entries down one spot.
	 * 
	 * @param dels
	 * @param b
	 * @param start
	 */
	private void cleanDelimiter(Delimiter[] dels, Body b, boolean start) {
		for (int i = 0; i < numBodies * 2; i++) {
			if (dels[i] == null) { // Reached the end of the array (possible
									// because one delimiter could already have
									// been removed.)
				assert (i == numBodies * 2 - 1);
				break;
			}
			if (dels[i].body == b && (start == dels[i].isStart)) {
				dels[i] = null;
			}
			if (i > 0 && dels[i - 1] == null) {
				dels[i - 1] = dels[i];
				dels[i] = null;
			}
		}
	}
	// Performs a quick sort on the array. This method is not used in normal
	// running time, only at initialization and whenenver bodies are
	// added/removed.
	private void sort() {
		quicksort(delX, 0, numBodies * 2 - 1);
		quicksort(delY, 0, numBodies * 2 - 1);
		for (int i = 0; i < numBodies * 2 - 1; i++) {
			assert (delX[i].pos <= delX[i + 1].pos);
			assert (delY[i].pos <= delY[i + 1].pos);
		}
	}
	private Delimiter[] quicksort(Delimiter[] delimits, int left, int right) {
		if (right < left)
			return delimits;
		int i = left, j = right;
		float pivot = delimits[(left + right) / 2].pos;
		Delimiter tmp;
		while (i <= j) {
			while (delimits[i].pos < pivot) {
				i++;
			}
			while (delimits[j].pos > pivot) {
				j--;
			}
			if (i <= j) {
				tmp = delimits[i];
				delimits[i] = delimits[j];
				delimits[j] = tmp;
				i++;
				j--;
			}
		}
		if (left < j)
			quicksort(delimits, left, j);
		if (i < right)
			quicksort(delimits, i, right);
		return delimits;
	}
	private void findPairsForNewBody(Body b) {
		// Set all relevant delFound to false
		for (int i = 0; i < b.delIdEndX; i++) {
			delX[i].body.flag = false;
		}
		// All bodies to the left of b are only pairs if there's one delimiter
		// to the left, not two
		for (int i = 0; i < b.delIdStartX; i++) {
			delX[i].body.flag = !b.flag; // If a body is found two times it is
											// not added to the pairs list
		}
		// Every body that has one delimiter to the left of b surrounds b
		for (int i = 0; i < b.delIdStartX; i++) {
			if (delX[i].body.flag) {
				addPairX(b, delX[i].body);
			}
		}
		// Every body that has any delimiter between the bodies bounds is added
		// as a pair (If a body gets added twice the addPair method will take
		// care of it)
		for (int i = b.delIdStartX + 1; i < b.delIdEndX; i++) {
			addPairX(b, delX[i].body);
		}
	}
	public void setPairArray(Pair[][] pairs) {
		pairArray = pairs;
	}
}
