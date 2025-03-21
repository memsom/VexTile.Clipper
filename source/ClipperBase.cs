namespace VexTile.ClipperLib;

public class ClipperBase
{
  protected const double horizontal = -3.4E+38;
  internal const long loRange = 1073741823;
  internal const long hiRange = 4611686018427387903;
  internal LocalMinima m_MinimaList;
  internal LocalMinima m_CurrentLM;
  internal List<List<TEdge>> m_edges = new List<List<TEdge>>();
  internal bool m_UseFullRange;

  protected static bool PointsEqual(IntPoint pt1, IntPoint pt2)
  {
    return pt1.X == pt2.X && pt1.Y == pt2.Y;
  }

  internal bool PointIsVertex(IntPoint pt, OutPt pp)
  {
    OutPt outPt = pp;
    while (!ClipperBase.PointsEqual(outPt.pt, pt))
    {
      outPt = outPt.next;
      if (outPt == pp)
        return false;
    }
    return true;
  }

  internal bool PointInPolygon(IntPoint pt, OutPt pp, bool UseFulllongRange)
  {
    OutPt outPt = pp;
    bool flag = false;
    if (UseFulllongRange)
    {
      do
      {
        if ((outPt.pt.Y <= pt.Y && pt.Y < outPt.prev.pt.Y || outPt.prev.pt.Y <= pt.Y && pt.Y < outPt.pt.Y) && new Int128(pt.X - outPt.pt.X) < Int128.Int128Mul(outPt.prev.pt.X - outPt.pt.X, pt.Y - outPt.pt.Y) / new Int128(outPt.prev.pt.Y - outPt.pt.Y))
          flag = !flag;
        outPt = outPt.next;
      }
      while (outPt != pp);
    }
    else
    {
      do
      {
        if ((outPt.pt.Y <= pt.Y && pt.Y < outPt.prev.pt.Y || outPt.prev.pt.Y <= pt.Y && pt.Y < outPt.pt.Y) && pt.X - outPt.pt.X < (outPt.prev.pt.X - outPt.pt.X) * (pt.Y - outPt.pt.Y) / (outPt.prev.pt.Y - outPt.pt.Y))
          flag = !flag;
        outPt = outPt.next;
      }
      while (outPt != pp);
    }
    return flag;
  }

  internal bool SlopesEqual(TEdge e1, TEdge e2, bool UseFullRange)
  {
    return UseFullRange ? Int128.Int128Mul(e1.deltaY, e2.deltaX) == Int128.Int128Mul(e1.deltaX, e2.deltaY) : e1.deltaY * e2.deltaX == e1.deltaX * e2.deltaY;
  }

  protected bool SlopesEqual(IntPoint pt1, IntPoint pt2, IntPoint pt3, bool UseFullRange)
  {
    return UseFullRange ? Int128.Int128Mul(pt1.Y - pt2.Y, pt2.X - pt3.X) == Int128.Int128Mul(pt1.X - pt2.X, pt2.Y - pt3.Y) : (pt1.Y - pt2.Y) * (pt2.X - pt3.X) - (pt1.X - pt2.X) * (pt2.Y - pt3.Y) == 0L;
  }

  protected bool SlopesEqual(
    IntPoint pt1,
    IntPoint pt2,
    IntPoint pt3,
    IntPoint pt4,
    bool UseFullRange)
  {
    return UseFullRange ? Int128.Int128Mul(pt1.Y - pt2.Y, pt3.X - pt4.X) == Int128.Int128Mul(pt1.X - pt2.X, pt3.Y - pt4.Y) : (pt1.Y - pt2.Y) * (pt3.X - pt4.X) - (pt1.X - pt2.X) * (pt3.Y - pt4.Y) == 0L;
  }

  internal ClipperBase()
  {
    this.m_MinimaList = (LocalMinima) null;
    this.m_CurrentLM = (LocalMinima) null;
    this.m_UseFullRange = false;
  }

  public virtual void Clear()
  {
    this.DisposeLocalMinimaList();
    for (int index1 = 0; index1 < this.m_edges.Count; ++index1)
    {
      for (int index2 = 0; index2 < this.m_edges[index1].Count; ++index2)
        this.m_edges[index1][index2] = (TEdge) null;
      this.m_edges[index1].Clear();
    }
    this.m_edges.Clear();
    this.m_UseFullRange = false;
  }

  private void DisposeLocalMinimaList()
  {
    LocalMinima next;
    for (; this.m_MinimaList != null; this.m_MinimaList = next)
    {
      next = this.m_MinimaList.next;
      this.m_MinimaList = (LocalMinima) null;
    }
    this.m_CurrentLM = (LocalMinima) null;
  }

  public bool AddPolygons(List<List<IntPoint>> ppg, PolyType polyType)
  {
    bool flag = false;
    for (int index = 0; index < ppg.Count; ++index)
    {
      if (this.AddPolygon(ppg[index], polyType))
        flag = true;
    }
    return flag;
  }

  public bool AddPolygon(List<IntPoint> pg, PolyType polyType)
  {
    int count = pg.Count;
    if (count < 3)
      return false;
    List<IntPoint> intPointList = new List<IntPoint>(count);
    intPointList.Add(new IntPoint(pg[0].X, pg[0].Y));
    int index1 = 0;
    for (int index2 = 1; index2 < count; ++index2)
    {
      long num = !this.m_UseFullRange ? 1073741823L : 4611686018427387903L;
      if (Math.Abs(pg[index2].X) > num || Math.Abs(pg[index2].Y) > num)
      {
        if (Math.Abs(pg[index2].X) > 4611686018427387903L || Math.Abs(pg[index2].Y) > 4611686018427387903L)
          throw new ClipperException("Coordinate exceeds range bounds");
        this.m_UseFullRange = true;
      }
      if (!ClipperBase.PointsEqual(intPointList[index1], pg[index2]))
      {
        if (index1 > 0 && this.SlopesEqual(intPointList[index1 - 1], intPointList[index1], pg[index2], this.m_UseFullRange))
        {
          if (ClipperBase.PointsEqual(intPointList[index1 - 1], pg[index2]))
            --index1;
        }
        else
          ++index1;
        if (index1 < intPointList.Count)
          intPointList[index1] = pg[index2];
        else
          intPointList.Add(new IntPoint(pg[index2].X, pg[index2].Y));
      }
    }
    if (index1 < 2)
      return false;
    int capacity;
    for (capacity = index1 + 1; capacity > 2; --capacity)
    {
      if (ClipperBase.PointsEqual(intPointList[index1], intPointList[0]))
        --index1;
      else if (ClipperBase.PointsEqual(intPointList[0], intPointList[1]) || this.SlopesEqual(intPointList[index1], intPointList[0], intPointList[1], this.m_UseFullRange))
        intPointList[0] = intPointList[index1--];
      else if (this.SlopesEqual(intPointList[index1 - 1], intPointList[index1], intPointList[0], this.m_UseFullRange))
        --index1;
      else if (this.SlopesEqual(intPointList[0], intPointList[1], intPointList[2], this.m_UseFullRange))
      {
        for (int index3 = 2; index3 <= index1; ++index3)
          intPointList[index3 - 1] = intPointList[index3];
        --index1;
      }
      else
        break;
    }
    if (capacity < 3)
      return false;
    List<TEdge> tedgeList = new List<TEdge>(capacity);
    for (int index4 = 0; index4 < capacity; ++index4)
      tedgeList.Add(new TEdge());
    this.m_edges.Add(tedgeList);
    tedgeList[0].xcurr = intPointList[0].X;
    tedgeList[0].ycurr = intPointList[0].Y;
    this.InitEdge(tedgeList[capacity - 1], tedgeList[0], tedgeList[capacity - 2], intPointList[capacity - 1], polyType);
    for (int index5 = capacity - 2; index5 > 0; --index5)
      this.InitEdge(tedgeList[index5], tedgeList[index5 + 1], tedgeList[index5 - 1], intPointList[index5], polyType);
    this.InitEdge(tedgeList[0], tedgeList[1], tedgeList[capacity - 1], intPointList[0], polyType);
    TEdge next = tedgeList[0];
    TEdge tedge = next;
    do
    {
      next.xcurr = next.xbot;
      next.ycurr = next.ybot;
      if (next.ytop < tedge.ytop)
        tedge = next;
      next = next.next;
    }
    while (next != tedgeList[0]);
    if (tedge.windDelta > 0)
      tedge = tedge.next;
    if (tedge.dx == -3.4E+38)
      tedge = tedge.next;
    TEdge e = tedge;
    do
    {
      e = this.AddBoundsToLML(e);
    }
    while (e != tedge);
    return true;
  }

  private void InitEdge(TEdge e, TEdge eNext, TEdge ePrev, IntPoint pt, PolyType polyType)
  {
    e.next = eNext;
    e.prev = ePrev;
    e.xcurr = pt.X;
    e.ycurr = pt.Y;
    if (e.ycurr >= e.next.ycurr)
    {
      e.xbot = e.xcurr;
      e.ybot = e.ycurr;
      e.xtop = e.next.xcurr;
      e.ytop = e.next.ycurr;
      e.windDelta = 1;
    }
    else
    {
      e.xtop = e.xcurr;
      e.ytop = e.ycurr;
      e.xbot = e.next.xcurr;
      e.ybot = e.next.ycurr;
      e.windDelta = -1;
    }
    this.SetDx(e);
    e.polyType = polyType;
    e.outIdx = -1;
  }

  private void SetDx(TEdge e)
  {
    e.deltaX = e.xtop - e.xbot;
    e.deltaY = e.ytop - e.ybot;
    if (e.deltaY == 0L)
      e.dx = -3.4E+38;
    else
      e.dx = (double) e.deltaX / (double) e.deltaY;
  }

  private TEdge AddBoundsToLML(TEdge e)
  {
    e.nextInLML = (TEdge) null;
    e = e.next;
    while (true)
    {
      if (e.dx == -3.4E+38)
      {
        if (e.next.ytop >= e.ytop || e.next.xbot <= e.prev.xbot)
        {
          if (e.xtop != e.prev.xbot)
            this.SwapX(e);
          e.nextInLML = e.prev;
        }
        else
          break;
      }
      else if (e.ycurr != e.prev.ycurr)
        e.nextInLML = e.prev;
      else
        break;
      e = e.next;
    }
    LocalMinima newLm = new LocalMinima();
    newLm.next = (LocalMinima) null;
    newLm.Y = e.prev.ybot;
    if (e.dx == -3.4E+38)
    {
      if (e.xbot != e.prev.xbot)
        this.SwapX(e);
      newLm.leftBound = e.prev;
      newLm.rightBound = e;
    }
    else if (e.dx < e.prev.dx)
    {
      newLm.leftBound = e.prev;
      newLm.rightBound = e;
    }
    else
    {
      newLm.leftBound = e;
      newLm.rightBound = e.prev;
    }
    newLm.leftBound.side = EdgeSide.esLeft;
    newLm.rightBound.side = EdgeSide.esRight;
    this.InsertLocalMinima(newLm);
    while (e.next.ytop != e.ytop || e.next.dx == -3.4E+38)
    {
      e.nextInLML = e.next;
      e = e.next;
      if (e.dx == -3.4E+38 && e.xbot != e.prev.xtop)
        this.SwapX(e);
    }
    return e.next;
  }

  private void InsertLocalMinima(LocalMinima newLm)
  {
    if (this.m_MinimaList == null)
      this.m_MinimaList = newLm;
    else if (newLm.Y >= this.m_MinimaList.Y)
    {
      newLm.next = this.m_MinimaList;
      this.m_MinimaList = newLm;
    }
    else
    {
      LocalMinima localMinima = this.m_MinimaList;
      while (localMinima.next != null && newLm.Y < localMinima.next.Y)
        localMinima = localMinima.next;
      newLm.next = localMinima.next;
      localMinima.next = newLm;
    }
  }

  protected void PopLocalMinima()
  {
    if (this.m_CurrentLM == null)
      return;
    this.m_CurrentLM = this.m_CurrentLM.next;
  }

  private void SwapX(TEdge e)
  {
    e.xcurr = e.xtop;
    e.xtop = e.xbot;
    e.xbot = e.xcurr;
  }

  protected virtual void Reset()
  {
    this.m_CurrentLM = this.m_MinimaList;
    for (LocalMinima localMinima = this.m_MinimaList; localMinima != null; localMinima = localMinima.next)
    {
      for (TEdge tedge = localMinima.leftBound; tedge != null; tedge = tedge.nextInLML)
      {
        tedge.xcurr = tedge.xbot;
        tedge.ycurr = tedge.ybot;
        tedge.side = EdgeSide.esLeft;
        tedge.outIdx = -1;
      }
      for (TEdge tedge = localMinima.rightBound; tedge != null; tedge = tedge.nextInLML)
      {
        tedge.xcurr = tedge.xbot;
        tedge.ycurr = tedge.ybot;
        tedge.side = EdgeSide.esRight;
        tedge.outIdx = -1;
      }
    }
  }

  public IntRect GetBounds()
  {
    IntRect bounds = new IntRect();
    LocalMinima localMinima = this.m_MinimaList;
    if (localMinima == null)
      return bounds;
    bounds.left = localMinima.leftBound.xbot;
    bounds.top = localMinima.leftBound.ybot;
    bounds.right = localMinima.leftBound.xbot;
    bounds.bottom = localMinima.leftBound.ybot;
    for (; localMinima != null; localMinima = localMinima.next)
    {
      if (localMinima.leftBound.ybot > bounds.bottom)
        bounds.bottom = localMinima.leftBound.ybot;
      TEdge tedge1 = localMinima.leftBound;
      while (true)
      {
        TEdge tedge2 = tedge1;
        for (; tedge1.nextInLML != null; tedge1 = tedge1.nextInLML)
        {
          if (tedge1.xbot < bounds.left)
            bounds.left = tedge1.xbot;
          if (tedge1.xbot > bounds.right)
            bounds.right = tedge1.xbot;
        }
        if (tedge1.xbot < bounds.left)
          bounds.left = tedge1.xbot;
        if (tedge1.xbot > bounds.right)
          bounds.right = tedge1.xbot;
        if (tedge1.xtop < bounds.left)
          bounds.left = tedge1.xtop;
        if (tedge1.xtop > bounds.right)
          bounds.right = tedge1.xtop;
        if (tedge1.ytop < bounds.top)
          bounds.top = tedge1.ytop;
        if (tedge2 == localMinima.leftBound)
          tedge1 = localMinima.rightBound;
        else
          break;
      }
    }
    return bounds;
  }
}