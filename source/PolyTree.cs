namespace VexTile.ClipperLib;

public class PolyTree : PolyNode
{
  internal List<PolyNode> m_AllPolys = new List<PolyNode>();

  ~PolyTree() => this.Clear();

  public void Clear()
  {
    for (int index = 0; index < this.m_AllPolys.Count; ++index)
      this.m_AllPolys[index] = (PolyNode) null;
    this.m_AllPolys.Clear();
    this.m_Childs.Clear();
  }

  public PolyNode GetFirst() => this.m_Childs.Count > 0 ? this.m_Childs[0] : (PolyNode) null;

  public int Total => this.m_AllPolys.Count;
}