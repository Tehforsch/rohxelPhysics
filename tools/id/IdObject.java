package tools.id;
public interface IdObject {
	int DEFAULTID = -1;
	public int getId();
	public void setId(int id);
	public void setIndexInList(int i);
}
