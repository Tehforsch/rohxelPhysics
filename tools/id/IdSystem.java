package tools.id;
import java.io.Serializable;
import java.util.Vector;
public class IdSystem<X extends IdObject> implements Serializable {
	private static final int EMPTYINDEX = -1;
	private Vector<X> objects;
	private Vector<Integer> idToIndex;
	private int nextId;
	public IdSystem() {
		this(0);
	}
	public IdSystem(int i) {
		super();
		nextId = i;
		this.objects = new Vector<X>();
		this.idToIndex = new Vector<Integer>();
	}
	private void update() {
		while (idToIndex.size() < objects.size()) { // Anzahl der Objekte
													// groesser als Anzahl der
													// Eintraege?
			idToIndex.add(IdObject.DEFAULTID); // Vergroessern
		}
		for (X obj : objects) {
			while (obj.getId() >= idToIndex.size()) { // Solange eine ID
														// existiert, die nicht
														// in dem array
														// enthalten sein kann :
				idToIndex.add(IdObject.DEFAULTID); // Vergroessern
			}
		}
		// Set all values to -1 so ids that dont exist any more wont have any
		// index assigned to them
		for (int i = 0; i < idToIndex.size(); i++) {
			idToIndex.set(i, EMPTYINDEX);
		}
		// Set the entries
		for (int i = 0; i < objects.size(); i++) {
			X obj = objects.get(i);
			int id = obj.getId();
			if (id == IdObject.DEFAULTID)
				continue;
			idToIndex.set(id, i);
			obj.setIndexInList(i);
		}
	}
	public void add(X obj) {
		objects.add(obj);
		if (obj.getId() == IdObject.DEFAULTID) { // Id is not yet set.
			obj.setId(getNextId());
		}
		else { // Id is set. Increase nextId if necessary
			if (nextId <= obj.getId()) {
				nextId = obj.getId() + 1;
			}
		}
		update();
	}
	public void remove(X obj) {
		objects.remove(obj);
		update();
	}
	public X getById(int id) {
		assert (id != IdObject.DEFAULTID) : "an object with the id" + IdObject.DEFAULTID + " never exists.";
		assert (idToIndex.get(id) != EMPTYINDEX) : "The object with the id " + id + " doesn't exist.";
		return objects.get(idToIndex.get(id));
	}
	public X get(int i) {
		return objects.get(i);
	}
	private int getNextId() {
		nextId++;
		return nextId - 1;
	}
	public int size() {
		return objects.size();
	}
	public boolean contains(X obj) {
		return objects.contains(obj);
	}
	public boolean containsId(int id) {
		if (id == IdObject.DEFAULTID)
			return false;
		return (id < idToIndex.size() && idToIndex.get(id) != EMPTYINDEX);
	}
	@Override
	public String toString() {
		String s = "[";
		for (X obj : objects) {
			s = s + obj.toString() + ",";
		}
		return s + "]";
	}
}
