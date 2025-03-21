namespace VexTile.ClipperLib;

public class Clipper : ClipperBase
{
  private List<OutRec> m_PolyOuts;
  private ClipType m_ClipType;
  private Scanbeam m_Scanbeam;
  private TEdge m_ActiveEdges;
  private TEdge m_SortedEdges;
  private IntersectNode m_IntersectNodes;
  private bool m_ExecuteLocked;
  private PolyFillType m_ClipFillType;
  private PolyFillType m_SubjFillType;
  private List<JoinRec> m_Joins;
  private List<HorzJoinRec> m_HorizJoins;
  private bool m_ReverseOutput;
  private bool m_UsingPolyTree;

  public Clipper()
  {
    this.m_Scanbeam = (Scanbeam) null;
    this.m_ActiveEdges = (TEdge) null;
    this.m_SortedEdges = (TEdge) null;
    this.m_IntersectNodes = (IntersectNode) null;
    this.m_ExecuteLocked = false;
    this.m_UsingPolyTree = false;
    this.m_PolyOuts = new List<OutRec>();
    this.m_Joins = new List<JoinRec>();
    this.m_HorizJoins = new List<HorzJoinRec>();
    this.m_ReverseOutput = false;
  }

  public override void Clear()
  {
    if (this.m_edges.Count == 0)
      return;
    this.DisposeAllPolyPts();
    base.Clear();
  }

  private void DisposeScanbeamList()
  {
    Scanbeam next;
    for (; this.m_Scanbeam != null; this.m_Scanbeam = next)
    {
      next = this.m_Scanbeam.next;
      this.m_Scanbeam = (Scanbeam) null;
    }
  }

  protected override void Reset()
  {
    base.Reset();
    this.m_Scanbeam = (Scanbeam) null;
    this.m_ActiveEdges = (TEdge) null;
    this.m_SortedEdges = (TEdge) null;
    this.DisposeAllPolyPts();
    for (LocalMinima localMinima = this.m_MinimaList; localMinima != null; localMinima = localMinima.next)
    {
      this.InsertScanbeam(localMinima.Y);
      this.InsertScanbeam(localMinima.leftBound.ytop);
    }
  }

  public bool ReverseSolution
  {
    get => this.m_ReverseOutput;
    set => this.m_ReverseOutput = value;
  }

  private void InsertScanbeam(long Y)
  {
    if (this.m_Scanbeam == null)
    {
      this.m_Scanbeam = new Scanbeam();
      this.m_Scanbeam.next = (Scanbeam) null;
      this.m_Scanbeam.Y = Y;
    }
    else if (Y > this.m_Scanbeam.Y)
    {
      this.m_Scanbeam = new Scanbeam()
      {
        Y = Y,
        next = this.m_Scanbeam
      };
    }
    else
    {
      Scanbeam scanbeam = this.m_Scanbeam;
      while (scanbeam.next != null && Y <= scanbeam.next.Y)
        scanbeam = scanbeam.next;
      if (Y == scanbeam.Y)
        return;
      scanbeam.next = new Scanbeam()
      {
        Y = Y,
        next = scanbeam.next
      };
    }
  }

  public bool Execute(
    ClipType clipType,
    List<List<IntPoint>> solution,
    PolyFillType subjFillType,
    PolyFillType clipFillType)
  {
    if (this.m_ExecuteLocked)
      return false;
    this.m_ExecuteLocked = true;
    solution.Clear();
    this.m_SubjFillType = subjFillType;
    this.m_ClipFillType = clipFillType;
    this.m_ClipType = clipType;
    this.m_UsingPolyTree = false;
    bool flag = this.ExecuteInternal();
    if (flag)
      this.BuildResult(solution);
    this.m_ExecuteLocked = false;
    return flag;
  }

  public bool Execute(
    ClipType clipType,
    PolyTree polytree,
    PolyFillType subjFillType,
    PolyFillType clipFillType)
  {
    if (this.m_ExecuteLocked)
      return false;
    this.m_ExecuteLocked = true;
    this.m_SubjFillType = subjFillType;
    this.m_ClipFillType = clipFillType;
    this.m_ClipType = clipType;
    this.m_UsingPolyTree = true;
    bool flag = this.ExecuteInternal();
    if (flag)
      this.BuildResult2(polytree);
    this.m_ExecuteLocked = false;
    return flag;
  }

  public bool Execute(ClipType clipType, List<List<IntPoint>> solution)
  {
    return this.Execute(clipType, solution, PolyFillType.pftEvenOdd, PolyFillType.pftEvenOdd);
  }

  public bool Execute(ClipType clipType, PolyTree polytree)
  {
    return this.Execute(clipType, polytree, PolyFillType.pftEvenOdd, PolyFillType.pftEvenOdd);
  }

  internal void FixHoleLinkage(OutRec outRec)
  {
    if (outRec.FirstLeft == null || outRec.isHole != outRec.FirstLeft.isHole && outRec.FirstLeft.pts != null)
      return;
    OutRec firstLeft = outRec.FirstLeft;
    while (firstLeft != null && (firstLeft.isHole == outRec.isHole || firstLeft.pts == null))
      firstLeft = firstLeft.FirstLeft;
    outRec.FirstLeft = firstLeft;
  }

  private bool ExecuteInternal()
  {
    bool flag;
    try
    {
      this.Reset();
      if (this.m_CurrentLM == null)
        return true;
      long botY = this.PopScanbeam();
      do
      {
        this.InsertLocalMinimaIntoAEL(botY);
        this.m_HorizJoins.Clear();
        this.ProcessHorizontals();
        long topY = this.PopScanbeam();
        flag = this.ProcessIntersections(botY, topY);
        if (flag)
        {
          this.ProcessEdgesAtTopOfScanbeam(topY);
          botY = topY;
        }
        else
          break;
      }
      while (this.m_Scanbeam != null);
    }
    catch
    {
      flag = false;
    }
    if (flag)
    {
      for (int index = 0; index < this.m_PolyOuts.Count; ++index)
      {
        OutRec polyOut = this.m_PolyOuts[index];
        if (polyOut.pts != null)
        {
          this.FixupOutPolygon(polyOut);
          if (polyOut.pts != null && (polyOut.isHole ^ this.m_ReverseOutput) == this.Area(polyOut, this.m_UseFullRange) > 0.0)
            this.ReversePolyPtLinks(polyOut.pts);
        }
      }
      this.JoinCommonEdges();
    }
    this.m_Joins.Clear();
    this.m_HorizJoins.Clear();
    return flag;
  }

  private long PopScanbeam()
  {
    long y = this.m_Scanbeam.Y;
    this.m_Scanbeam = this.m_Scanbeam.next;
    return y;
  }

  private void DisposeAllPolyPts()
  {
    for (int index = 0; index < this.m_PolyOuts.Count; ++index)
      this.DisposeOutRec(index);
    this.m_PolyOuts.Clear();
  }

  private void DisposeOutRec(int index)
  {
    OutRec polyOut = this.m_PolyOuts[index];
    if (polyOut.pts != null)
      this.DisposeOutPts(polyOut.pts);
    this.m_PolyOuts[index] = (OutRec) null;
  }

  private void DisposeOutPts(OutPt pp)
  {
    if (pp == null)
      return;
    pp.prev.next = (OutPt) null;
    while (pp != null)
      pp = pp.next;
  }

  private void AddJoin(TEdge e1, TEdge e2, int e1OutIdx, int e2OutIdx)
  {
    this.m_Joins.Add(new JoinRec()
    {
      poly1Idx = e1OutIdx < 0 ? e1.outIdx : e1OutIdx,
      pt1a = new IntPoint(e1.xcurr, e1.ycurr),
      pt1b = new IntPoint(e1.xtop, e1.ytop),
      poly2Idx = e2OutIdx < 0 ? e2.outIdx : e2OutIdx,
      pt2a = new IntPoint(e2.xcurr, e2.ycurr),
      pt2b = new IntPoint(e2.xtop, e2.ytop)
    });
  }

  private void AddHorzJoin(TEdge e, int idx)
  {
    this.m_HorizJoins.Add(new HorzJoinRec()
    {
      edge = e,
      savedIdx = idx
    });
  }

  private void InsertLocalMinimaIntoAEL(long botY)
  {
    while (this.m_CurrentLM != null && this.m_CurrentLM.Y == botY)
    {
      TEdge leftBound = this.m_CurrentLM.leftBound;
      TEdge rightBound = this.m_CurrentLM.rightBound;
      this.InsertEdgeIntoAEL(leftBound);
      this.InsertScanbeam(leftBound.ytop);
      this.InsertEdgeIntoAEL(rightBound);
      if (this.IsEvenOddFillType(leftBound))
      {
        leftBound.windDelta = 1;
        rightBound.windDelta = 1;
      }
      else
        rightBound.windDelta = -leftBound.windDelta;
      this.SetWindingCount(leftBound);
      rightBound.windCnt = leftBound.windCnt;
      rightBound.windCnt2 = leftBound.windCnt2;
      if (rightBound.dx == -3.4E+38)
      {
        this.AddEdgeToSEL(rightBound);
        this.InsertScanbeam(rightBound.nextInLML.ytop);
      }
      else
        this.InsertScanbeam(rightBound.ytop);
      if (this.IsContributing(leftBound))
        this.AddLocalMinPoly(leftBound, rightBound, new IntPoint(leftBound.xcurr, this.m_CurrentLM.Y));
      if (rightBound.outIdx >= 0 && rightBound.dx == -3.4E+38)
      {
        for (int index = 0; index < this.m_HorizJoins.Count; ++index)
        {
          IntPoint pt1 = new IntPoint();
          IntPoint pt2 = new IntPoint();
          HorzJoinRec horizJoin = this.m_HorizJoins[index];
          if (this.GetOverlapSegment(new IntPoint(horizJoin.edge.xbot, horizJoin.edge.ybot), new IntPoint(horizJoin.edge.xtop, horizJoin.edge.ytop), new IntPoint(rightBound.xbot, rightBound.ybot), new IntPoint(rightBound.xtop, rightBound.ytop), ref pt1, ref pt2))
            this.AddJoin(horizJoin.edge, rightBound, horizJoin.savedIdx, -1);
        }
      }
      if (leftBound.nextInAEL != rightBound)
      {
        if (rightBound.outIdx >= 0 && rightBound.prevInAEL.outIdx >= 0 && this.SlopesEqual(rightBound.prevInAEL, rightBound, this.m_UseFullRange))
          this.AddJoin(rightBound, rightBound.prevInAEL, -1, -1);
        TEdge nextInAel = leftBound.nextInAEL;
        IntPoint pt = new IntPoint(leftBound.xcurr, leftBound.ycurr);
        for (; nextInAel != rightBound; nextInAel = nextInAel.nextInAEL)
        {
          if (nextInAel == null)
            throw new ClipperException("InsertLocalMinimaIntoAEL: missing rightbound!");
          this.IntersectEdges(rightBound, nextInAel, pt, Protects.ipNone);
        }
      }
      this.PopLocalMinima();
    }
  }

  private void InsertEdgeIntoAEL(TEdge edge)
  {
    edge.prevInAEL = (TEdge) null;
    edge.nextInAEL = (TEdge) null;
    if (this.m_ActiveEdges == null)
      this.m_ActiveEdges = edge;
    else if (this.E2InsertsBeforeE1(this.m_ActiveEdges, edge))
    {
      edge.nextInAEL = this.m_ActiveEdges;
      this.m_ActiveEdges.prevInAEL = edge;
      this.m_ActiveEdges = edge;
    }
    else
    {
      TEdge tedge = this.m_ActiveEdges;
      while (tedge.nextInAEL != null && !this.E2InsertsBeforeE1(tedge.nextInAEL, edge))
        tedge = tedge.nextInAEL;
      edge.nextInAEL = tedge.nextInAEL;
      if (tedge.nextInAEL != null)
        tedge.nextInAEL.prevInAEL = edge;
      edge.prevInAEL = tedge;
      tedge.nextInAEL = edge;
    }
  }

  private bool E2InsertsBeforeE1(TEdge e1, TEdge e2)
  {
    return e2.xcurr != e1.xcurr ? e2.xcurr < e1.xcurr : e2.dx > e1.dx;
  }

  private bool IsEvenOddFillType(TEdge edge)
  {
    return edge.polyType == PolyType.ptSubject ? this.m_SubjFillType == PolyFillType.pftEvenOdd : this.m_ClipFillType == PolyFillType.pftEvenOdd;
  }

  private bool IsEvenOddAltFillType(TEdge edge)
  {
    return edge.polyType == PolyType.ptSubject ? this.m_ClipFillType == PolyFillType.pftEvenOdd : this.m_SubjFillType == PolyFillType.pftEvenOdd;
  }

  private bool IsContributing(TEdge edge)
  {
    PolyFillType polyFillType1;
    PolyFillType polyFillType2;
    if (edge.polyType == PolyType.ptSubject)
    {
      polyFillType1 = this.m_SubjFillType;
      polyFillType2 = this.m_ClipFillType;
    }
    else
    {
      polyFillType1 = this.m_ClipFillType;
      polyFillType2 = this.m_SubjFillType;
    }
    switch (polyFillType1)
    {
      case PolyFillType.pftEvenOdd:
      case PolyFillType.pftNonZero:
        if (Math.Abs(edge.windCnt) != 1)
          return false;
        break;
      case PolyFillType.pftPositive:
        if (edge.windCnt != 1)
          return false;
        break;
      default:
        if (edge.windCnt != -1)
          return false;
        break;
    }
    switch (this.m_ClipType)
    {
      case ClipType.ctIntersection:
        switch (polyFillType2)
        {
          case PolyFillType.pftEvenOdd:
          case PolyFillType.pftNonZero:
            return edge.windCnt2 != 0;
          case PolyFillType.pftPositive:
            return edge.windCnt2 > 0;
          default:
            return edge.windCnt2 < 0;
        }
      case ClipType.ctUnion:
        switch (polyFillType2)
        {
          case PolyFillType.pftEvenOdd:
          case PolyFillType.pftNonZero:
            return edge.windCnt2 == 0;
          case PolyFillType.pftPositive:
            return edge.windCnt2 <= 0;
          default:
            return edge.windCnt2 >= 0;
        }
      case ClipType.ctDifference:
        if (edge.polyType == PolyType.ptSubject)
        {
          switch (polyFillType2)
          {
            case PolyFillType.pftEvenOdd:
            case PolyFillType.pftNonZero:
              return edge.windCnt2 == 0;
            case PolyFillType.pftPositive:
              return edge.windCnt2 <= 0;
            default:
              return edge.windCnt2 >= 0;
          }
        }
        else
        {
          switch (polyFillType2)
          {
            case PolyFillType.pftEvenOdd:
            case PolyFillType.pftNonZero:
              return edge.windCnt2 != 0;
            case PolyFillType.pftPositive:
              return edge.windCnt2 > 0;
            default:
              return edge.windCnt2 < 0;
          }
        }
      default:
        return true;
    }
  }

  private void SetWindingCount(TEdge edge)
  {
    TEdge prevInAel = edge.prevInAEL;
    while (prevInAel != null && prevInAel.polyType != edge.polyType)
      prevInAel = prevInAel.prevInAEL;
    TEdge tedge;
    if (prevInAel == null)
    {
      edge.windCnt = edge.windDelta;
      edge.windCnt2 = 0;
      tedge = this.m_ActiveEdges;
    }
    else if (this.IsEvenOddFillType(edge))
    {
      edge.windCnt = 1;
      edge.windCnt2 = prevInAel.windCnt2;
      tedge = prevInAel.nextInAEL;
    }
    else
    {
      edge.windCnt = prevInAel.windCnt * prevInAel.windDelta >= 0 ? (Math.Abs(prevInAel.windCnt) <= 1 || prevInAel.windDelta * edge.windDelta >= 0 ? (prevInAel.windCnt + edge.windDelta != 0 ? prevInAel.windCnt + edge.windDelta : prevInAel.windCnt) : prevInAel.windCnt) : (Math.Abs(prevInAel.windCnt) <= 1 ? prevInAel.windCnt + prevInAel.windDelta + edge.windDelta : (prevInAel.windDelta * edge.windDelta >= 0 ? prevInAel.windCnt + edge.windDelta : prevInAel.windCnt));
      edge.windCnt2 = prevInAel.windCnt2;
      tedge = prevInAel.nextInAEL;
    }
    if (this.IsEvenOddAltFillType(edge))
    {
      for (; tedge != edge; tedge = tedge.nextInAEL)
        edge.windCnt2 = edge.windCnt2 == 0 ? 1 : 0;
    }
    else
    {
      for (; tedge != edge; tedge = tedge.nextInAEL)
        edge.windCnt2 += tedge.windDelta;
    }
  }

  private void AddEdgeToSEL(TEdge edge)
  {
    if (this.m_SortedEdges == null)
    {
      this.m_SortedEdges = edge;
      edge.prevInSEL = (TEdge) null;
      edge.nextInSEL = (TEdge) null;
    }
    else
    {
      edge.nextInSEL = this.m_SortedEdges;
      edge.prevInSEL = (TEdge) null;
      this.m_SortedEdges.prevInSEL = edge;
      this.m_SortedEdges = edge;
    }
  }

  private void CopyAELToSEL()
  {
    TEdge activeEdges = this.m_ActiveEdges;
    this.m_SortedEdges = activeEdges;
    if (this.m_ActiveEdges == null)
      return;
    this.m_SortedEdges.prevInSEL = (TEdge) null;
    for (TEdge nextInAel = activeEdges.nextInAEL; nextInAel != null; nextInAel = nextInAel.nextInAEL)
    {
      nextInAel.prevInSEL = nextInAel.prevInAEL;
      nextInAel.prevInSEL.nextInSEL = nextInAel;
      nextInAel.nextInSEL = (TEdge) null;
    }
  }

  private void SwapPositionsInAEL(TEdge edge1, TEdge edge2)
  {
    if (edge1.nextInAEL == edge2)
    {
      TEdge nextInAel = edge2.nextInAEL;
      if (nextInAel != null)
        nextInAel.prevInAEL = edge1;
      TEdge prevInAel = edge1.prevInAEL;
      if (prevInAel != null)
        prevInAel.nextInAEL = edge2;
      edge2.prevInAEL = prevInAel;
      edge2.nextInAEL = edge1;
      edge1.prevInAEL = edge2;
      edge1.nextInAEL = nextInAel;
    }
    else if (edge2.nextInAEL == edge1)
    {
      TEdge nextInAel = edge1.nextInAEL;
      if (nextInAel != null)
        nextInAel.prevInAEL = edge2;
      TEdge prevInAel = edge2.prevInAEL;
      if (prevInAel != null)
        prevInAel.nextInAEL = edge1;
      edge1.prevInAEL = prevInAel;
      edge1.nextInAEL = edge2;
      edge2.prevInAEL = edge1;
      edge2.nextInAEL = nextInAel;
    }
    else
    {
      TEdge nextInAel = edge1.nextInAEL;
      TEdge prevInAel = edge1.prevInAEL;
      edge1.nextInAEL = edge2.nextInAEL;
      if (edge1.nextInAEL != null)
        edge1.nextInAEL.prevInAEL = edge1;
      edge1.prevInAEL = edge2.prevInAEL;
      if (edge1.prevInAEL != null)
        edge1.prevInAEL.nextInAEL = edge1;
      edge2.nextInAEL = nextInAel;
      if (edge2.nextInAEL != null)
        edge2.nextInAEL.prevInAEL = edge2;
      edge2.prevInAEL = prevInAel;
      if (edge2.prevInAEL != null)
        edge2.prevInAEL.nextInAEL = edge2;
    }
    if (edge1.prevInAEL == null)
    {
      this.m_ActiveEdges = edge1;
    }
    else
    {
      if (edge2.prevInAEL != null)
        return;
      this.m_ActiveEdges = edge2;
    }
  }

  private void SwapPositionsInSEL(TEdge edge1, TEdge edge2)
  {
    if (edge1.nextInSEL == null && edge1.prevInSEL == null || edge2.nextInSEL == null && edge2.prevInSEL == null)
      return;
    if (edge1.nextInSEL == edge2)
    {
      TEdge nextInSel = edge2.nextInSEL;
      if (nextInSel != null)
        nextInSel.prevInSEL = edge1;
      TEdge prevInSel = edge1.prevInSEL;
      if (prevInSel != null)
        prevInSel.nextInSEL = edge2;
      edge2.prevInSEL = prevInSel;
      edge2.nextInSEL = edge1;
      edge1.prevInSEL = edge2;
      edge1.nextInSEL = nextInSel;
    }
    else if (edge2.nextInSEL == edge1)
    {
      TEdge nextInSel = edge1.nextInSEL;
      if (nextInSel != null)
        nextInSel.prevInSEL = edge2;
      TEdge prevInSel = edge2.prevInSEL;
      if (prevInSel != null)
        prevInSel.nextInSEL = edge1;
      edge1.prevInSEL = prevInSel;
      edge1.nextInSEL = edge2;
      edge2.prevInSEL = edge1;
      edge2.nextInSEL = nextInSel;
    }
    else
    {
      TEdge nextInSel = edge1.nextInSEL;
      TEdge prevInSel = edge1.prevInSEL;
      edge1.nextInSEL = edge2.nextInSEL;
      if (edge1.nextInSEL != null)
        edge1.nextInSEL.prevInSEL = edge1;
      edge1.prevInSEL = edge2.prevInSEL;
      if (edge1.prevInSEL != null)
        edge1.prevInSEL.nextInSEL = edge1;
      edge2.nextInSEL = nextInSel;
      if (edge2.nextInSEL != null)
        edge2.nextInSEL.prevInSEL = edge2;
      edge2.prevInSEL = prevInSel;
      if (edge2.prevInSEL != null)
        edge2.prevInSEL.nextInSEL = edge2;
    }
    if (edge1.prevInSEL == null)
    {
      this.m_SortedEdges = edge1;
    }
    else
    {
      if (edge2.prevInSEL != null)
        return;
      this.m_SortedEdges = edge2;
    }
  }

  private void AddLocalMaxPoly(TEdge e1, TEdge e2, IntPoint pt)
  {
    this.AddOutPt(e1, pt);
    if (e1.outIdx == e2.outIdx)
    {
      e1.outIdx = -1;
      e2.outIdx = -1;
    }
    else if (e1.outIdx < e2.outIdx)
      this.AppendPolygon(e1, e2);
    else
      this.AppendPolygon(e2, e1);
  }

  private void AddLocalMinPoly(TEdge e1, TEdge e2, IntPoint pt)
  {
    TEdge tedge1;
    TEdge tedge2;
    if (e2.dx == -3.4E+38 || e1.dx > e2.dx)
    {
      this.AddOutPt(e1, pt);
      e2.outIdx = e1.outIdx;
      e1.side = EdgeSide.esLeft;
      e2.side = EdgeSide.esRight;
      tedge1 = e1;
      tedge2 = tedge1.prevInAEL != e2 ? tedge1.prevInAEL : e2.prevInAEL;
    }
    else
    {
      this.AddOutPt(e2, pt);
      e1.outIdx = e2.outIdx;
      e1.side = EdgeSide.esRight;
      e2.side = EdgeSide.esLeft;
      tedge1 = e2;
      tedge2 = tedge1.prevInAEL != e1 ? tedge1.prevInAEL : e1.prevInAEL;
    }
    if (tedge2 == null || tedge2.outIdx < 0 || Clipper.TopX(tedge2, pt.Y) != Clipper.TopX(tedge1, pt.Y) || !this.SlopesEqual(tedge1, tedge2, this.m_UseFullRange))
      return;
    this.AddJoin(tedge1, tedge2, -1, -1);
  }

  private OutRec CreateOutRec()
  {
    return new OutRec()
    {
      idx = -1,
      isHole = false,
      FirstLeft = (OutRec) null,
      pts = (OutPt) null,
      bottomPt = (OutPt) null,
      polyNode = (PolyNode) null
    };
  }

  private void AddOutPt(TEdge e, IntPoint pt)
  {
    bool flag = e.side == EdgeSide.esLeft;
    if (e.outIdx < 0)
    {
      OutRec outRec = this.CreateOutRec();
      this.m_PolyOuts.Add(outRec);
      outRec.idx = this.m_PolyOuts.Count - 1;
      e.outIdx = outRec.idx;
      OutPt outPt = new OutPt();
      outRec.pts = outPt;
      outRec.bottomPt = outPt;
      outPt.pt = pt;
      outPt.idx = outRec.idx;
      outPt.next = outPt;
      outPt.prev = outPt;
      this.SetHoleState(e, outRec);
    }
    else
    {
      OutRec polyOut = this.m_PolyOuts[e.outIdx];
      OutPt pts = polyOut.pts;
      if (flag && ClipperBase.PointsEqual(pt, pts.pt) || !flag && ClipperBase.PointsEqual(pt, pts.prev.pt))
        return;
      OutPt outPt = new OutPt();
      outPt.pt = pt;
      outPt.idx = polyOut.idx;
      if (outPt.pt.Y == polyOut.bottomPt.pt.Y && outPt.pt.X < polyOut.bottomPt.pt.X)
        polyOut.bottomPt = outPt;
      outPt.next = pts;
      outPt.prev = pts.prev;
      outPt.prev.next = outPt;
      pts.prev = outPt;
      if (!flag)
        return;
      polyOut.pts = outPt;
    }
  }

  internal void SwapPoints(ref IntPoint pt1, ref IntPoint pt2)
  {
    IntPoint intPoint = pt1;
    pt1 = pt2;
    pt2 = intPoint;
  }

  private bool GetOverlapSegment(
    IntPoint pt1a,
    IntPoint pt1b,
    IntPoint pt2a,
    IntPoint pt2b,
    ref IntPoint pt1,
    ref IntPoint pt2)
  {
    if (Math.Abs(pt1a.X - pt1b.X) > Math.Abs(pt1a.Y - pt1b.Y))
    {
      if (pt1a.X > pt1b.X)
        this.SwapPoints(ref pt1a, ref pt1b);
      if (pt2a.X > pt2b.X)
        this.SwapPoints(ref pt2a, ref pt2b);
      pt1 = pt1a.X <= pt2a.X ? pt2a : pt1a;
      pt2 = pt1b.X >= pt2b.X ? pt2b : pt1b;
      return pt1.X < pt2.X;
    }
    if (pt1a.Y < pt1b.Y)
      this.SwapPoints(ref pt1a, ref pt1b);
    if (pt2a.Y < pt2b.Y)
      this.SwapPoints(ref pt2a, ref pt2b);
    pt1 = pt1a.Y >= pt2a.Y ? pt2a : pt1a;
    pt2 = pt1b.Y <= pt2b.Y ? pt2b : pt1b;
    return pt1.Y > pt2.Y;
  }

  private bool FindSegment(ref OutPt pp, ref IntPoint pt1, ref IntPoint pt2)
  {
    if (pp == null)
      return false;
    OutPt outPt = pp;
    IntPoint intPoint1 = new IntPoint(pt1);
    IntPoint intPoint2 = new IntPoint(pt2);
    while (!this.SlopesEqual(intPoint1, intPoint2, pp.pt, pp.prev.pt, true) || !this.SlopesEqual(intPoint1, intPoint2, pp.pt, true) || !this.GetOverlapSegment(intPoint1, intPoint2, pp.pt, pp.prev.pt, ref pt1, ref pt2))
    {
      pp = pp.next;
      if (pp == outPt)
        return false;
    }
    return true;
  }

  internal bool Pt3IsBetweenPt1AndPt2(IntPoint pt1, IntPoint pt2, IntPoint pt3)
  {
    if (ClipperBase.PointsEqual(pt1, pt3) || ClipperBase.PointsEqual(pt2, pt3))
      return true;
    return pt1.X != pt2.X ? pt1.X < pt3.X == pt3.X < pt2.X : pt1.Y < pt3.Y == pt3.Y < pt2.Y;
  }

  private OutPt InsertPolyPtBetween(OutPt p1, OutPt p2, IntPoint pt)
  {
    OutPt outPt = new OutPt();
    outPt.pt = pt;
    if (p2 == p1.next)
    {
      p1.next = outPt;
      p2.prev = outPt;
      outPt.next = p2;
      outPt.prev = p1;
    }
    else
    {
      p2.next = outPt;
      p1.prev = outPt;
      outPt.next = p1;
      outPt.prev = p2;
    }
    return outPt;
  }

  private void SetHoleState(TEdge e, OutRec outRec)
  {
    bool flag = false;
    for (TEdge prevInAel = e.prevInAEL; prevInAel != null; prevInAel = prevInAel.prevInAEL)
    {
      if (prevInAel.outIdx >= 0)
      {
        flag = !flag;
        if (outRec.FirstLeft == null)
          outRec.FirstLeft = this.m_PolyOuts[prevInAel.outIdx];
      }
    }
    if (!flag)
      return;
    outRec.isHole = true;
  }

  private double GetDx(IntPoint pt1, IntPoint pt2)
  {
    return pt1.Y == pt2.Y ? -3.4E+38 : (double) (pt2.X - pt1.X) / (double) (pt2.Y - pt1.Y);
  }

  private bool FirstIsBottomPt(OutPt btmPt1, OutPt btmPt2)
  {
    OutPt prev1 = btmPt1.prev;
    while (ClipperBase.PointsEqual(prev1.pt, btmPt1.pt) && prev1 != btmPt1)
      prev1 = prev1.prev;
    double num1 = Math.Abs(this.GetDx(btmPt1.pt, prev1.pt));
    OutPt next1 = btmPt1.next;
    while (ClipperBase.PointsEqual(next1.pt, btmPt1.pt) && next1 != btmPt1)
      next1 = next1.next;
    double num2 = Math.Abs(this.GetDx(btmPt1.pt, next1.pt));
    OutPt prev2 = btmPt2.prev;
    while (ClipperBase.PointsEqual(prev2.pt, btmPt2.pt) && prev2 != btmPt2)
      prev2 = prev2.prev;
    double num3 = Math.Abs(this.GetDx(btmPt2.pt, prev2.pt));
    OutPt next2 = btmPt2.next;
    while (ClipperBase.PointsEqual(next2.pt, btmPt2.pt) && next2 != btmPt2)
      next2 = next2.next;
    double num4 = Math.Abs(this.GetDx(btmPt2.pt, next2.pt));
    if (num1 >= num3 && num1 >= num4)
      return true;
    return num2 >= num3 && num2 >= num4;
  }

  private OutPt GetBottomPt(OutPt pp)
  {
    OutPt btmPt2 = (OutPt) null;
    OutPt next;
    for (next = pp.next; next != pp; next = next.next)
    {
      if (next.pt.Y > pp.pt.Y)
      {
        pp = next;
        btmPt2 = (OutPt) null;
      }
      else if (next.pt.Y == pp.pt.Y && next.pt.X <= pp.pt.X)
      {
        if (next.pt.X < pp.pt.X)
        {
          btmPt2 = (OutPt) null;
          pp = next;
        }
        else if (next.next != pp && next.prev != pp)
          btmPt2 = next;
      }
    }
    if (btmPt2 != null)
    {
      while (btmPt2 != next)
      {
        if (!this.FirstIsBottomPt(next, btmPt2))
          pp = btmPt2;
        btmPt2 = btmPt2.next;
        while (!ClipperBase.PointsEqual(btmPt2.pt, pp.pt))
          btmPt2 = btmPt2.next;
      }
    }
    return pp;
  }

  private OutRec GetLowermostRec(OutRec outRec1, OutRec outRec2)
  {
    OutPt bottomPt1 = outRec1.bottomPt;
    OutPt bottomPt2 = outRec2.bottomPt;
    return bottomPt1.pt.Y > bottomPt2.pt.Y || bottomPt1.pt.Y >= bottomPt2.pt.Y && (bottomPt1.pt.X < bottomPt2.pt.X || bottomPt1.pt.X <= bottomPt2.pt.X && bottomPt1.next != bottomPt1 && (bottomPt2.next == bottomPt2 || this.FirstIsBottomPt(bottomPt1, bottomPt2))) ? outRec1 : outRec2;
  }

  private bool Param1RightOfParam2(OutRec outRec1, OutRec outRec2)
  {
    do
    {
      outRec1 = outRec1.FirstLeft;
      if (outRec1 == outRec2)
        return true;
    }
    while (outRec1 != null);
    return false;
  }

  private void AppendPolygon(TEdge e1, TEdge e2)
  {
    OutRec polyOut1 = this.m_PolyOuts[e1.outIdx];
    OutRec polyOut2 = this.m_PolyOuts[e2.outIdx];
    OutRec outRec = !this.Param1RightOfParam2(polyOut1, polyOut2) ? (!this.Param1RightOfParam2(polyOut2, polyOut1) ? this.GetLowermostRec(polyOut1, polyOut2) : polyOut1) : polyOut2;
    OutPt pts1 = polyOut1.pts;
    OutPt prev1 = pts1.prev;
    OutPt pts2 = polyOut2.pts;
    OutPt prev2 = pts2.prev;
    EdgeSide edgeSide;
    if (e1.side == EdgeSide.esLeft)
    {
      if (e2.side == EdgeSide.esLeft)
      {
        this.ReversePolyPtLinks(pts2);
        pts2.next = pts1;
        pts1.prev = pts2;
        prev1.next = prev2;
        prev2.prev = prev1;
        polyOut1.pts = prev2;
      }
      else
      {
        prev2.next = pts1;
        pts1.prev = prev2;
        pts2.prev = prev1;
        prev1.next = pts2;
        polyOut1.pts = pts2;
      }
      edgeSide = EdgeSide.esLeft;
    }
    else
    {
      if (e2.side == EdgeSide.esRight)
      {
        this.ReversePolyPtLinks(pts2);
        prev1.next = prev2;
        prev2.prev = prev1;
        pts2.next = pts1;
        pts1.prev = pts2;
      }
      else
      {
        prev1.next = pts2;
        pts2.prev = prev1;
        pts1.prev = prev2;
        prev2.next = pts1;
      }
      edgeSide = EdgeSide.esRight;
    }
    if (outRec == polyOut2)
    {
      polyOut1.bottomPt = polyOut2.bottomPt;
      polyOut1.bottomPt.idx = polyOut1.idx;
      if (polyOut2.FirstLeft != polyOut1)
        polyOut1.FirstLeft = polyOut2.FirstLeft;
      polyOut1.isHole = polyOut2.isHole;
    }
    polyOut2.pts = (OutPt) null;
    polyOut2.bottomPt = (OutPt) null;
    polyOut2.FirstLeft = polyOut1;
    int outIdx1 = e1.outIdx;
    int outIdx2 = e2.outIdx;
    e1.outIdx = -1;
    e2.outIdx = -1;
    for (TEdge tedge = this.m_ActiveEdges; tedge != null; tedge = tedge.nextInAEL)
    {
      if (tedge.outIdx == outIdx2)
      {
        tedge.outIdx = outIdx1;
        tedge.side = edgeSide;
        break;
      }
    }
    for (int index = 0; index < this.m_Joins.Count; ++index)
    {
      if (this.m_Joins[index].poly1Idx == outIdx2)
        this.m_Joins[index].poly1Idx = outIdx1;
      if (this.m_Joins[index].poly2Idx == outIdx2)
        this.m_Joins[index].poly2Idx = outIdx1;
    }
    for (int index = 0; index < this.m_HorizJoins.Count; ++index)
    {
      if (this.m_HorizJoins[index].savedIdx == outIdx2)
        this.m_HorizJoins[index].savedIdx = outIdx1;
    }
  }

  private void ReversePolyPtLinks(OutPt pp)
  {
    if (pp == null)
      return;
    OutPt outPt = pp;
    do
    {
      OutPt next = outPt.next;
      outPt.next = outPt.prev;
      outPt.prev = next;
      outPt = next;
    }
    while (outPt != pp);
  }

  private static void SwapSides(TEdge edge1, TEdge edge2)
  {
    EdgeSide side = edge1.side;
    edge1.side = edge2.side;
    edge2.side = side;
  }

  private static void SwapPolyIndexes(TEdge edge1, TEdge edge2)
  {
    int outIdx = edge1.outIdx;
    edge1.outIdx = edge2.outIdx;
    edge2.outIdx = outIdx;
  }

  private void DoEdge1(TEdge edge1, TEdge edge2, IntPoint pt)
  {
    this.AddOutPt(edge1, pt);
    Clipper.SwapSides(edge1, edge2);
    Clipper.SwapPolyIndexes(edge1, edge2);
  }

  private void DoEdge2(TEdge edge1, TEdge edge2, IntPoint pt)
  {
    this.AddOutPt(edge2, pt);
    Clipper.SwapSides(edge1, edge2);
    Clipper.SwapPolyIndexes(edge1, edge2);
  }

  private void DoBothEdges(TEdge edge1, TEdge edge2, IntPoint pt)
  {
    this.AddOutPt(edge1, pt);
    this.AddOutPt(edge2, pt);
    Clipper.SwapSides(edge1, edge2);
    Clipper.SwapPolyIndexes(edge1, edge2);
  }

  private void IntersectEdges(TEdge e1, TEdge e2, IntPoint pt, Protects protects)
  {
    bool flag1 = (Protects.ipLeft & protects) == Protects.ipNone && e1.nextInLML == null && e1.xtop == pt.X && e1.ytop == pt.Y;
    bool flag2 = (Protects.ipRight & protects) == Protects.ipNone && e2.nextInLML == null && e2.xtop == pt.X && e2.ytop == pt.Y;
    bool flag3 = e1.outIdx >= 0;
    bool flag4 = e2.outIdx >= 0;
    if (e1.polyType == e2.polyType)
    {
      if (this.IsEvenOddFillType(e1))
      {
        int windCnt = e1.windCnt;
        e1.windCnt = e2.windCnt;
        e2.windCnt = windCnt;
      }
      else
      {
        if (e1.windCnt + e2.windDelta == 0)
          e1.windCnt = -e1.windCnt;
        else
          e1.windCnt += e2.windDelta;
        if (e2.windCnt - e1.windDelta == 0)
          e2.windCnt = -e2.windCnt;
        else
          e2.windCnt -= e1.windDelta;
      }
    }
    else
    {
      if (!this.IsEvenOddFillType(e2))
        e1.windCnt2 += e2.windDelta;
      else
        e1.windCnt2 = e1.windCnt2 == 0 ? 1 : 0;
      if (!this.IsEvenOddFillType(e1))
        e2.windCnt2 -= e1.windDelta;
      else
        e2.windCnt2 = e2.windCnt2 == 0 ? 1 : 0;
    }
    PolyFillType polyFillType1;
    PolyFillType polyFillType2;
    if (e1.polyType == PolyType.ptSubject)
    {
      polyFillType1 = this.m_SubjFillType;
      polyFillType2 = this.m_ClipFillType;
    }
    else
    {
      polyFillType1 = this.m_ClipFillType;
      polyFillType2 = this.m_SubjFillType;
    }
    PolyFillType polyFillType3;
    PolyFillType polyFillType4;
    if (e2.polyType == PolyType.ptSubject)
    {
      polyFillType3 = this.m_SubjFillType;
      polyFillType4 = this.m_ClipFillType;
    }
    else
    {
      polyFillType3 = this.m_ClipFillType;
      polyFillType4 = this.m_SubjFillType;
    }
    int num1;
    switch (polyFillType1)
    {
      case PolyFillType.pftPositive:
        num1 = e1.windCnt;
        break;
      case PolyFillType.pftNegative:
        num1 = -e1.windCnt;
        break;
      default:
        num1 = Math.Abs(e1.windCnt);
        break;
    }
    int num2;
    switch (polyFillType3)
    {
      case PolyFillType.pftPositive:
        num2 = e2.windCnt;
        break;
      case PolyFillType.pftNegative:
        num2 = -e2.windCnt;
        break;
      default:
        num2 = Math.Abs(e2.windCnt);
        break;
    }
    if (flag3 && flag4)
    {
      if (flag1 || flag2 || num1 != 0 && num1 != 1 || num2 != 0 && num2 != 1 || e1.polyType != e2.polyType && this.m_ClipType != ClipType.ctXor)
        this.AddLocalMaxPoly(e1, e2, pt);
      else
        this.DoBothEdges(e1, e2, pt);
    }
    else if (flag3)
    {
      if ((num2 == 0 || num2 == 1) && (this.m_ClipType != ClipType.ctIntersection || e2.polyType == PolyType.ptSubject || e2.windCnt2 != 0))
        this.DoEdge1(e1, e2, pt);
    }
    else if (flag4)
    {
      if ((num1 == 0 || num1 == 1) && (this.m_ClipType != ClipType.ctIntersection || e1.polyType == PolyType.ptSubject || e1.windCnt2 != 0))
        this.DoEdge2(e1, e2, pt);
    }
    else if ((num1 == 0 || num1 == 1) && (num2 == 0 || num2 == 1) && !flag1 && !flag2)
    {
      long num3;
      switch (polyFillType2)
      {
        case PolyFillType.pftPositive:
          num3 = (long) e1.windCnt2;
          break;
        case PolyFillType.pftNegative:
          num3 = (long) -e1.windCnt2;
          break;
        default:
          num3 = (long) Math.Abs(e1.windCnt2);
          break;
      }
      long num4;
      switch (polyFillType4)
      {
        case PolyFillType.pftPositive:
          num4 = (long) e2.windCnt2;
          break;
        case PolyFillType.pftNegative:
          num4 = (long) -e2.windCnt2;
          break;
        default:
          num4 = (long) Math.Abs(e2.windCnt2);
          break;
      }
      if (e1.polyType != e2.polyType)
        this.AddLocalMinPoly(e1, e2, pt);
      else if (num1 == 1 && num2 == 1)
      {
        switch (this.m_ClipType)
        {
          case ClipType.ctIntersection:
            if (num3 > 0L && num4 > 0L)
            {
              this.AddLocalMinPoly(e1, e2, pt);
              break;
            }
            break;
          case ClipType.ctUnion:
            if (num3 <= 0L && num4 <= 0L)
            {
              this.AddLocalMinPoly(e1, e2, pt);
              break;
            }
            break;
          case ClipType.ctDifference:
            if (e1.polyType == PolyType.ptClip && num3 > 0L && num4 > 0L || e1.polyType == PolyType.ptSubject && num3 <= 0L && num4 <= 0L)
            {
              this.AddLocalMinPoly(e1, e2, pt);
              break;
            }
            break;
          case ClipType.ctXor:
            this.AddLocalMinPoly(e1, e2, pt);
            break;
        }
      }
      else
        Clipper.SwapSides(e1, e2);
    }
    if (flag1 != flag2 && (flag1 && e1.outIdx >= 0 || flag2 && e2.outIdx >= 0))
    {
      Clipper.SwapSides(e1, e2);
      Clipper.SwapPolyIndexes(e1, e2);
    }
    if (flag1)
      this.DeleteFromAEL(e1);
    if (!flag2)
      return;
    this.DeleteFromAEL(e2);
  }

  private void DeleteFromAEL(TEdge e)
  {
    TEdge prevInAel = e.prevInAEL;
    TEdge nextInAel = e.nextInAEL;
    if (prevInAel == null && nextInAel == null && e != this.m_ActiveEdges)
      return;
    if (prevInAel != null)
      prevInAel.nextInAEL = nextInAel;
    else
      this.m_ActiveEdges = nextInAel;
    if (nextInAel != null)
      nextInAel.prevInAEL = prevInAel;
    e.nextInAEL = (TEdge) null;
    e.prevInAEL = (TEdge) null;
  }

  private void DeleteFromSEL(TEdge e)
  {
    TEdge prevInSel = e.prevInSEL;
    TEdge nextInSel = e.nextInSEL;
    if (prevInSel == null && nextInSel == null && e != this.m_SortedEdges)
      return;
    if (prevInSel != null)
      prevInSel.nextInSEL = nextInSel;
    else
      this.m_SortedEdges = nextInSel;
    if (nextInSel != null)
      nextInSel.prevInSEL = prevInSel;
    e.nextInSEL = (TEdge) null;
    e.prevInSEL = (TEdge) null;
  }

  private void UpdateEdgeIntoAEL(ref TEdge e)
  {
    if (e.nextInLML == null)
      throw new ClipperException("UpdateEdgeIntoAEL: invalid call");
    TEdge prevInAel = e.prevInAEL;
    TEdge nextInAel = e.nextInAEL;
    e.nextInLML.outIdx = e.outIdx;
    if (prevInAel != null)
      prevInAel.nextInAEL = e.nextInLML;
    else
      this.m_ActiveEdges = e.nextInLML;
    if (nextInAel != null)
      nextInAel.prevInAEL = e.nextInLML;
    e.nextInLML.side = e.side;
    e.nextInLML.windDelta = e.windDelta;
    e.nextInLML.windCnt = e.windCnt;
    e.nextInLML.windCnt2 = e.windCnt2;
    e = e.nextInLML;
    e.prevInAEL = prevInAel;
    e.nextInAEL = nextInAel;
    if (e.dx == -3.4E+38)
      return;
    this.InsertScanbeam(e.ytop);
  }

  private void ProcessHorizontals()
  {
    for (TEdge sortedEdges = this.m_SortedEdges; sortedEdges != null; sortedEdges = this.m_SortedEdges)
    {
      this.DeleteFromSEL(sortedEdges);
      this.ProcessHorizontal(sortedEdges);
    }
  }

  private void ProcessHorizontal(TEdge horzEdge)
  {
    long num1;
    long num2;
    Direction Direction;
    if (horzEdge.xcurr < horzEdge.xtop)
    {
      num1 = horzEdge.xcurr;
      num2 = horzEdge.xtop;
      Direction = Direction.dLeftToRight;
    }
    else
    {
      num1 = horzEdge.xtop;
      num2 = horzEdge.xcurr;
      Direction = Direction.dRightToLeft;
    }
    TEdge maximaPair = horzEdge.nextInLML == null ? this.GetMaximaPair(horzEdge) : (TEdge) null;
    TEdge nextInAel;
    for (TEdge tedge = this.GetNextInAEL(horzEdge, Direction); tedge != null; tedge = nextInAel)
    {
      nextInAel = this.GetNextInAEL(tedge, Direction);
      if (maximaPair != null || Direction == Direction.dLeftToRight && tedge.xcurr <= num2 || Direction == Direction.dRightToLeft && tedge.xcurr >= num1)
      {
        if (tedge.xcurr == horzEdge.xtop && maximaPair == null)
        {
          if (this.SlopesEqual(tedge, horzEdge.nextInLML, this.m_UseFullRange))
          {
            if (horzEdge.outIdx >= 0 && tedge.outIdx >= 0)
            {
              this.AddJoin(horzEdge.nextInLML, tedge, horzEdge.outIdx, -1);
              break;
            }
            break;
          }
          if (tedge.dx < horzEdge.nextInLML.dx)
            break;
        }
        if (tedge == maximaPair)
        {
          if (Direction == Direction.dLeftToRight)
            this.IntersectEdges(horzEdge, tedge, new IntPoint(tedge.xcurr, horzEdge.ycurr), Protects.ipNone);
          else
            this.IntersectEdges(tedge, horzEdge, new IntPoint(tedge.xcurr, horzEdge.ycurr), Protects.ipNone);
          if (maximaPair.outIdx < 0)
            return;
          throw new ClipperException("ProcessHorizontal error");
        }
        if (tedge.dx == -3.4E+38 && !this.IsMinima(tedge) && tedge.xcurr <= tedge.xtop)
        {
          if (Direction == Direction.dLeftToRight)
            this.IntersectEdges(horzEdge, tedge, new IntPoint(tedge.xcurr, horzEdge.ycurr), this.IsTopHorz(horzEdge, (double) tedge.xcurr) ? Protects.ipLeft : Protects.ipBoth);
          else
            this.IntersectEdges(tedge, horzEdge, new IntPoint(tedge.xcurr, horzEdge.ycurr), this.IsTopHorz(horzEdge, (double) tedge.xcurr) ? Protects.ipRight : Protects.ipBoth);
        }
        else if (Direction == Direction.dLeftToRight)
          this.IntersectEdges(horzEdge, tedge, new IntPoint(tedge.xcurr, horzEdge.ycurr), this.IsTopHorz(horzEdge, (double) tedge.xcurr) ? Protects.ipLeft : Protects.ipBoth);
        else
          this.IntersectEdges(tedge, horzEdge, new IntPoint(tedge.xcurr, horzEdge.ycurr), this.IsTopHorz(horzEdge, (double) tedge.xcurr) ? Protects.ipRight : Protects.ipBoth);
        this.SwapPositionsInAEL(horzEdge, tedge);
      }
      else if (Direction == Direction.dLeftToRight && tedge.xcurr > num2 && horzEdge.nextInSEL == null || Direction == Direction.dRightToLeft && tedge.xcurr < num1 && horzEdge.nextInSEL == null)
        break;
    }
    if (horzEdge.nextInLML != null)
    {
      if (horzEdge.outIdx >= 0)
        this.AddOutPt(horzEdge, new IntPoint(horzEdge.xtop, horzEdge.ytop));
      this.UpdateEdgeIntoAEL(ref horzEdge);
    }
    else
    {
      if (horzEdge.outIdx >= 0)
        this.IntersectEdges(horzEdge, maximaPair, new IntPoint(horzEdge.xtop, horzEdge.ycurr), Protects.ipBoth);
      this.DeleteFromAEL(maximaPair);
      this.DeleteFromAEL(horzEdge);
    }
  }

  private bool IsTopHorz(TEdge horzEdge, double XPos)
  {
    for (TEdge tedge = this.m_SortedEdges; tedge != null; tedge = tedge.nextInSEL)
    {
      if (XPos >= (double) Math.Min(tedge.xcurr, tedge.xtop) && XPos <= (double) Math.Max(tedge.xcurr, tedge.xtop))
        return false;
    }
    return true;
  }

  private TEdge GetNextInAEL(TEdge e, Direction Direction)
  {
    return Direction != Direction.dLeftToRight ? e.prevInAEL : e.nextInAEL;
  }

  private bool IsMinima(TEdge e) => e != null && e.prev.nextInLML != e && e.next.nextInLML != e;

  private bool IsMaxima(TEdge e, double Y)
  {
    return e != null && (double) e.ytop == Y && e.nextInLML == null;
  }

  private bool IsIntermediate(TEdge e, double Y) => (double) e.ytop == Y && e.nextInLML != null;

  private TEdge GetMaximaPair(TEdge e)
  {
    return !this.IsMaxima(e.next, (double) e.ytop) || e.next.xtop != e.xtop ? e.prev : e.next;
  }

  private bool ProcessIntersections(long botY, long topY)
  {
    if (this.m_ActiveEdges == null)
      return true;
    try
    {
      this.BuildIntersectList(botY, topY);
      if (this.m_IntersectNodes == null)
        return true;
      if (!this.FixupIntersections())
        return false;
      this.ProcessIntersectList();
    }
    catch
    {
      this.m_SortedEdges = (TEdge) null;
      this.DisposeIntersectNodes();
      throw new ClipperException("ProcessIntersections error");
    }
    return true;
  }

  private void BuildIntersectList(long botY, long topY)
  {
    if (this.m_ActiveEdges == null)
      return;
    TEdge edge = this.m_ActiveEdges;
    this.m_SortedEdges = edge;
    for (; edge != null; edge = edge.nextInAEL)
    {
      edge.prevInSEL = edge.prevInAEL;
      edge.nextInSEL = edge.nextInAEL;
      edge.tmpX = Clipper.TopX(edge, topY);
    }
    bool flag = true;
    while (flag && this.m_SortedEdges != null)
    {
      flag = false;
      TEdge tedge = this.m_SortedEdges;
      while (tedge.nextInSEL != null)
      {
        TEdge nextInSel = tedge.nextInSEL;
        IntPoint ip = new IntPoint();
        if (tedge.tmpX > nextInSel.tmpX && this.IntersectPoint(tedge, nextInSel, ref ip))
        {
          if (ip.Y > botY)
          {
            ip.Y = botY;
            ip.X = Clipper.TopX(tedge, ip.Y);
          }
          this.AddIntersectNode(tedge, nextInSel, ip);
          this.SwapPositionsInSEL(tedge, nextInSel);
          flag = true;
        }
        else
          tedge = nextInSel;
      }
      if (tedge.prevInSEL != null)
        tedge.prevInSEL.nextInSEL = (TEdge) null;
      else
        break;
    }
    this.m_SortedEdges = (TEdge) null;
  }

  private bool FixupIntersections()
  {
    if (this.m_IntersectNodes.next == null)
      return true;
    this.CopyAELToSEL();
    IntersectNode int1 = this.m_IntersectNodes;
    for (IntersectNode next = this.m_IntersectNodes.next; next != null; next = int1.next)
    {
      TEdge edge1 = int1.edge1;
      TEdge edge2;
      if (edge1.prevInSEL == int1.edge2)
        edge2 = edge1.prevInSEL;
      else if (edge1.nextInSEL == int1.edge2)
      {
        edge2 = edge1.nextInSEL;
      }
      else
      {
        while (next != null && next.edge1.nextInSEL != next.edge2 && next.edge1.prevInSEL != next.edge2)
          next = next.next;
        if (next == null)
          return false;
        this.SwapIntersectNodes(int1, next);
        edge1 = int1.edge1;
        edge2 = int1.edge2;
      }
      this.SwapPositionsInSEL(edge1, edge2);
      int1 = int1.next;
    }
    this.m_SortedEdges = (TEdge) null;
    return int1.edge1.prevInSEL == int1.edge2 || int1.edge1.nextInSEL == int1.edge2;
  }

  private void ProcessIntersectList()
  {
    IntersectNode next;
    for (; this.m_IntersectNodes != null; this.m_IntersectNodes = next)
    {
      next = this.m_IntersectNodes.next;
      this.IntersectEdges(this.m_IntersectNodes.edge1, this.m_IntersectNodes.edge2, this.m_IntersectNodes.pt, Protects.ipBoth);
      this.SwapPositionsInAEL(this.m_IntersectNodes.edge1, this.m_IntersectNodes.edge2);
      this.m_IntersectNodes = (IntersectNode) null;
    }
  }

  private static long Round(double value)
  {
    return value >= 0.0 ? (long) (value + 0.5) : (long) (value - 0.5);
  }

  private static long TopX(TEdge edge, long currentY)
  {
    return currentY == edge.ytop ? edge.xtop : edge.xbot + Clipper.Round(edge.dx * (double) (currentY - edge.ybot));
  }

  private void AddIntersectNode(TEdge e1, TEdge e2, IntPoint pt)
  {
    IntersectNode intersectNode1 = new IntersectNode();
    intersectNode1.edge1 = e1;
    intersectNode1.edge2 = e2;
    intersectNode1.pt = pt;
    intersectNode1.next = (IntersectNode) null;
    if (this.m_IntersectNodes == null)
      this.m_IntersectNodes = intersectNode1;
    else if (this.ProcessParam1BeforeParam2(intersectNode1, this.m_IntersectNodes))
    {
      intersectNode1.next = this.m_IntersectNodes;
      this.m_IntersectNodes = intersectNode1;
    }
    else
    {
      IntersectNode intersectNode2 = this.m_IntersectNodes;
      while (intersectNode2.next != null && this.ProcessParam1BeforeParam2(intersectNode2.next, intersectNode1))
        intersectNode2 = intersectNode2.next;
      intersectNode1.next = intersectNode2.next;
      intersectNode2.next = intersectNode1;
    }
  }

  private bool ProcessParam1BeforeParam2(IntersectNode node1, IntersectNode node2)
  {
    if (node1.pt.Y != node2.pt.Y)
      return node1.pt.Y > node2.pt.Y;
    if (node1.edge1 == node2.edge1 || node1.edge2 == node2.edge1)
    {
      bool flag = node2.pt.X > node1.pt.X;
      return node2.edge1.dx <= 0.0 ? flag : !flag;
    }
    if (node1.edge1 != node2.edge2 && node1.edge2 != node2.edge2)
      return node2.pt.X > node1.pt.X;
    bool flag1 = node2.pt.X > node1.pt.X;
    return node2.edge2.dx <= 0.0 ? flag1 : !flag1;
  }

  private void SwapIntersectNodes(IntersectNode int1, IntersectNode int2)
  {
    TEdge edge1 = int1.edge1;
    TEdge edge2 = int1.edge2;
    IntPoint pt = int1.pt;
    int1.edge1 = int2.edge1;
    int1.edge2 = int2.edge2;
    int1.pt = int2.pt;
    int2.edge1 = edge1;
    int2.edge2 = edge2;
    int2.pt = pt;
  }

  private bool IntersectPoint(TEdge edge1, TEdge edge2, ref IntPoint ip)
  {
    if (this.SlopesEqual(edge1, edge2, this.m_UseFullRange))
      return false;
    if (edge1.dx == 0.0)
    {
      ip.X = edge1.xbot;
      if (edge2.dx == -3.4E+38)
      {
        ip.Y = edge2.ybot;
      }
      else
      {
        double num = (double) edge2.ybot - (double) edge2.xbot / edge2.dx;
        ip.Y = Clipper.Round((double) ip.X / edge2.dx + num);
      }
    }
    else if (edge2.dx == 0.0)
    {
      ip.X = edge2.xbot;
      if (edge1.dx == -3.4E+38)
      {
        ip.Y = edge1.ybot;
      }
      else
      {
        double num = (double) edge1.ybot - (double) edge1.xbot / edge1.dx;
        ip.Y = Clipper.Round((double) ip.X / edge1.dx + num);
      }
    }
    else
    {
      double num1 = (double) edge1.xbot - (double) edge1.ybot * edge1.dx;
      double num2 = (double) edge2.xbot - (double) edge2.ybot * edge2.dx;
      double num3 = (num2 - num1) / (edge1.dx - edge2.dx);
      ip.Y = Clipper.Round(num3);
      ip.X = Math.Abs(edge1.dx) >= Math.Abs(edge2.dx) ? Clipper.Round(edge2.dx * num3 + num2) : Clipper.Round(edge1.dx * num3 + num1);
    }
    if (ip.Y >= edge1.ytop && ip.Y >= edge2.ytop)
      return true;
    if (edge1.ytop > edge2.ytop)
    {
      ip.X = edge1.xtop;
      ip.Y = edge1.ytop;
      return Clipper.TopX(edge2, edge1.ytop) < edge1.xtop;
    }
    ip.X = edge2.xtop;
    ip.Y = edge2.ytop;
    return Clipper.TopX(edge1, edge2.ytop) > edge2.xtop;
  }

  private void DisposeIntersectNodes()
  {
    IntersectNode next;
    for (; this.m_IntersectNodes != null; this.m_IntersectNodes = next)
    {
      next = this.m_IntersectNodes.next;
      this.m_IntersectNodes = (IntersectNode) null;
    }
  }

  private void ProcessEdgesAtTopOfScanbeam(long topY)
  {
    TEdge e1 = this.m_ActiveEdges;
    while (e1 != null)
    {
      if (this.IsMaxima(e1, (double) topY) && this.GetMaximaPair(e1).dx != -3.4E+38)
      {
        TEdge prevInAel = e1.prevInAEL;
        this.DoMaxima(e1, topY);
        e1 = prevInAel != null ? prevInAel.nextInAEL : this.m_ActiveEdges;
      }
      else
      {
        if (this.IsIntermediate(e1, (double) topY) && e1.nextInLML.dx == -3.4E+38)
        {
          if (e1.outIdx >= 0)
          {
            this.AddOutPt(e1, new IntPoint(e1.xtop, e1.ytop));
            for (int index = 0; index < this.m_HorizJoins.Count; ++index)
            {
              IntPoint pt1 = new IntPoint();
              IntPoint pt2 = new IntPoint();
              HorzJoinRec horizJoin = this.m_HorizJoins[index];
              if (this.GetOverlapSegment(new IntPoint(horizJoin.edge.xbot, horizJoin.edge.ybot), new IntPoint(horizJoin.edge.xtop, horizJoin.edge.ytop), new IntPoint(e1.nextInLML.xbot, e1.nextInLML.ybot), new IntPoint(e1.nextInLML.xtop, e1.nextInLML.ytop), ref pt1, ref pt2))
                this.AddJoin(horizJoin.edge, e1.nextInLML, horizJoin.savedIdx, e1.outIdx);
            }
            this.AddHorzJoin(e1.nextInLML, e1.outIdx);
          }
          this.UpdateEdgeIntoAEL(ref e1);
          this.AddEdgeToSEL(e1);
        }
        else
        {
          e1.xcurr = Clipper.TopX(e1, topY);
          e1.ycurr = topY;
        }
        e1 = e1.nextInAEL;
      }
    }
    this.ProcessHorizontals();
    for (TEdge e2 = this.m_ActiveEdges; e2 != null; e2 = e2.nextInAEL)
    {
      if (this.IsIntermediate(e2, (double) topY))
      {
        if (e2.outIdx >= 0)
          this.AddOutPt(e2, new IntPoint(e2.xtop, e2.ytop));
        this.UpdateEdgeIntoAEL(ref e2);
        TEdge prevInAel = e2.prevInAEL;
        TEdge nextInAel = e2.nextInAEL;
        if (prevInAel != null && prevInAel.xcurr == e2.xbot && prevInAel.ycurr == e2.ybot && e2.outIdx >= 0 && prevInAel.outIdx >= 0 && prevInAel.ycurr > prevInAel.ytop && this.SlopesEqual(e2, prevInAel, this.m_UseFullRange))
        {
          this.AddOutPt(prevInAel, new IntPoint(e2.xbot, e2.ybot));
          this.AddJoin(e2, prevInAel, -1, -1);
        }
        else if (nextInAel != null && nextInAel.xcurr == e2.xbot && nextInAel.ycurr == e2.ybot && e2.outIdx >= 0 && nextInAel.outIdx >= 0 && nextInAel.ycurr > nextInAel.ytop && this.SlopesEqual(e2, nextInAel, this.m_UseFullRange))
        {
          this.AddOutPt(nextInAel, new IntPoint(e2.xbot, e2.ybot));
          this.AddJoin(e2, nextInAel, -1, -1);
        }
      }
    }
  }

  private void DoMaxima(TEdge e, long topY)
  {
    TEdge maximaPair = this.GetMaximaPair(e);
    long xtop = e.xtop;
    for (TEdge nextInAel = e.nextInAEL; nextInAel != maximaPair; nextInAel = nextInAel.nextInAEL)
    {
      if (nextInAel == null)
        throw new ClipperException("DoMaxima error");
      this.IntersectEdges(e, nextInAel, new IntPoint(xtop, topY), Protects.ipBoth);
      this.SwapPositionsInAEL(e, nextInAel);
    }
    if (e.outIdx < 0 && maximaPair.outIdx < 0)
    {
      this.DeleteFromAEL(e);
      this.DeleteFromAEL(maximaPair);
    }
    else
    {
      if (e.outIdx < 0 || maximaPair.outIdx < 0)
        throw new ClipperException("DoMaxima error");
      this.IntersectEdges(e, maximaPair, new IntPoint(xtop, topY), Protects.ipNone);
    }
  }

  public static void ReversePolygons(List<List<IntPoint>> polys)
  {
    polys.ForEach((Action<List<IntPoint>>) (poly => poly.Reverse()));
  }

  public static bool Orientation(List<IntPoint> poly) => Clipper.Area(poly) >= 0.0;

  private int PointCount(OutPt pts)
  {
    if (pts == null)
      return 0;
    int num = 0;
    OutPt outPt = pts;
    do
    {
      ++num;
      outPt = outPt.next;
    }
    while (outPt != pts);
    return num;
  }

  private void BuildResult(List<List<IntPoint>> polyg)
  {
    polyg.Clear();
    polyg.Capacity = this.m_PolyOuts.Count;
    for (int index1 = 0; index1 < this.m_PolyOuts.Count; ++index1)
    {
      OutRec polyOut = this.m_PolyOuts[index1];
      if (polyOut.pts != null)
      {
        OutPt pts = polyOut.pts;
        int capacity = this.PointCount(pts);
        if (capacity >= 3)
        {
          List<IntPoint> intPointList = new List<IntPoint>(capacity);
          for (int index2 = 0; index2 < capacity; ++index2)
          {
            intPointList.Add(pts.pt);
            pts = pts.prev;
          }
          polyg.Add(intPointList);
        }
      }
    }
  }

  private void BuildResult2(PolyTree polytree)
  {
    polytree.Clear();
    polytree.m_AllPolys.Capacity = this.m_PolyOuts.Count;
    for (int index1 = 0; index1 < this.m_PolyOuts.Count; ++index1)
    {
      OutRec polyOut = this.m_PolyOuts[index1];
      int num = this.PointCount(polyOut.pts);
      if (num >= 3)
      {
        this.FixHoleLinkage(polyOut);
        PolyNode polyNode = new PolyNode();
        polytree.m_AllPolys.Add(polyNode);
        polyOut.polyNode = polyNode;
        polyNode.m_polygon.Capacity = num;
        OutPt outPt = polyOut.pts;
        for (int index2 = 0; index2 < num; ++index2)
        {
          polyNode.m_polygon.Add(outPt.pt);
          outPt = outPt.prev;
        }
      }
    }
    polytree.m_Childs.Capacity = this.m_PolyOuts.Count;
    for (int index = 0; index < this.m_PolyOuts.Count; ++index)
    {
      OutRec polyOut = this.m_PolyOuts[index];
      if (polyOut.polyNode != null)
      {
        if (polyOut.FirstLeft == null)
        {
          polyOut.polyNode.m_Index = polytree.m_Childs.Count;
          polytree.m_Childs.Add(polyOut.polyNode);
          polyOut.polyNode.m_Parent = (PolyNode) polytree;
        }
        else
          polyOut.FirstLeft.polyNode.AddChild(polyOut.polyNode);
      }
    }
  }

  private void FixupOutPolygon(OutRec outRec)
  {
    OutPt outPt = (OutPt) null;
    outRec.pts = outRec.bottomPt;
    OutPt pp = outRec.bottomPt;
    while (pp.prev != pp && pp.prev != pp.next)
    {
      if (ClipperBase.PointsEqual(pp.pt, pp.next.pt) || this.SlopesEqual(pp.prev.pt, pp.pt, pp.next.pt, this.m_UseFullRange))
      {
        outPt = (OutPt) null;
        if (pp == outRec.bottomPt)
          outRec.bottomPt = (OutPt) null;
        pp.prev.next = pp.next;
        pp.next.prev = pp.prev;
        pp = pp.prev;
      }
      else if (pp != outPt)
      {
        if (outPt == null)
          outPt = pp;
        pp = pp.next;
      }
      else
      {
        if (outRec.bottomPt != null)
          return;
        outRec.bottomPt = this.GetBottomPt(pp);
        outRec.bottomPt.idx = outRec.idx;
        outRec.pts = outRec.bottomPt;
        return;
      }
    }
    this.DisposeOutPts(pp);
    outRec.pts = (OutPt) null;
    outRec.bottomPt = (OutPt) null;
  }

  private bool JoinPoints(JoinRec j, out OutPt p1, out OutPt p2)
  {
    p1 = (OutPt) null;
    p2 = (OutPt) null;
    OutRec polyOut1 = this.m_PolyOuts[j.poly1Idx];
    OutRec polyOut2 = this.m_PolyOuts[j.poly2Idx];
    if (polyOut1 == null || polyOut2 == null)
      return false;
    OutPt pts = polyOut1.pts;
    OutPt pp = polyOut2.pts;
    IntPoint pt2a = j.pt2a;
    IntPoint pt2b = j.pt2b;
    IntPoint pt1a = j.pt1a;
    IntPoint pt1b = j.pt1b;
    if (!this.FindSegment(ref pts, ref pt2a, ref pt2b))
      return false;
    if (polyOut1 == polyOut2)
    {
      pp = pts.next;
      if (!this.FindSegment(ref pp, ref pt1a, ref pt1b) || pp == pts)
        return false;
    }
    else if (!this.FindSegment(ref pp, ref pt1a, ref pt1b))
      return false;
    if (!this.GetOverlapSegment(pt2a, pt2b, pt1a, pt1b, ref pt2a, ref pt2b))
      return false;
    OutPt prev1 = pts.prev;
    p1 = !ClipperBase.PointsEqual(pts.pt, pt2a) ? (!ClipperBase.PointsEqual(prev1.pt, pt2a) ? this.InsertPolyPtBetween(pts, prev1, pt2a) : prev1) : pts;
    p2 = !ClipperBase.PointsEqual(pts.pt, pt2b) ? (!ClipperBase.PointsEqual(prev1.pt, pt2b) ? (p1 == pts || p1 == prev1 ? this.InsertPolyPtBetween(pts, prev1, pt2b) : (!this.Pt3IsBetweenPt1AndPt2(pts.pt, p1.pt, pt2b) ? this.InsertPolyPtBetween(p1, prev1, pt2b) : this.InsertPolyPtBetween(pts, p1, pt2b))) : prev1) : pts;
    OutPt prev2 = pp.prev;
    OutPt outPt1 = !ClipperBase.PointsEqual(pp.pt, pt2a) ? (!ClipperBase.PointsEqual(prev2.pt, pt2a) ? this.InsertPolyPtBetween(pp, prev2, pt2a) : prev2) : pp;
    OutPt outPt2 = !ClipperBase.PointsEqual(pp.pt, pt2b) ? (!ClipperBase.PointsEqual(prev2.pt, pt2b) ? (outPt1 == pp || outPt1 == prev2 ? this.InsertPolyPtBetween(pp, prev2, pt2b) : (!this.Pt3IsBetweenPt1AndPt2(pp.pt, outPt1.pt, pt2b) ? this.InsertPolyPtBetween(outPt1, prev2, pt2b) : this.InsertPolyPtBetween(pp, outPt1, pt2b))) : prev2) : pp;
    if (p1.next == p2 && outPt1.prev == outPt2)
    {
      p1.next = outPt1;
      outPt1.prev = p1;
      p2.prev = outPt2;
      outPt2.next = p2;
      return true;
    }
    if (p1.prev != p2 || outPt1.next != outPt2)
      return false;
    p1.prev = outPt1;
    outPt1.next = p1;
    p2.next = outPt2;
    outPt2.prev = p2;
    return true;
  }

  private void FixupJoinRecs(JoinRec j, OutPt pt, int startIdx)
  {
    for (int index = startIdx; index < this.m_Joins.Count; ++index)
    {
      JoinRec join = this.m_Joins[index];
      if (join.poly1Idx == j.poly1Idx && this.PointIsVertex(join.pt1a, pt))
        join.poly1Idx = j.poly2Idx;
      if (join.poly2Idx == j.poly1Idx && this.PointIsVertex(join.pt2a, pt))
        join.poly2Idx = j.poly2Idx;
    }
  }

  private bool Poly2ContainsPoly1(OutPt outPt1, OutPt outPt2, bool UseFullInt64Range)
  {
    OutPt outPt = outPt1;
    while (this.PointIsVertex(outPt.pt, outPt2))
    {
      outPt = outPt.next;
      if (outPt == outPt1)
        break;
    }
    bool flag;
    do
    {
      flag = this.PointInPolygon(outPt.pt, outPt2, UseFullInt64Range);
      outPt = outPt.next;
    }
    while (flag && outPt != outPt1);
    return flag;
  }

  private void FixupFirstLefts1(OutRec OldOutRec, OutRec NewOutRec)
  {
    for (int index = 0; index < this.m_PolyOuts.Count; ++index)
    {
      OutRec polyOut = this.m_PolyOuts[index];
      if (polyOut.pts != null && polyOut.FirstLeft == OldOutRec && this.Poly2ContainsPoly1(polyOut.pts, NewOutRec.pts, this.m_UseFullRange))
        polyOut.FirstLeft = NewOutRec;
    }
  }

  private void FixupFirstLefts2(OutRec OldOutRec, OutRec NewOutRec)
  {
    foreach (OutRec polyOut in this.m_PolyOuts)
    {
      if (polyOut.FirstLeft == OldOutRec)
        polyOut.FirstLeft = NewOutRec;
    }
  }

  private void JoinCommonEdges()
  {
    for (int index1 = 0; index1 < this.m_Joins.Count; ++index1)
    {
      JoinRec join1 = this.m_Joins[index1];
      OutRec polyOut1 = this.m_PolyOuts[join1.poly1Idx];
      OutRec polyOut2 = this.m_PolyOuts[join1.poly2Idx];
      if (polyOut1.pts != null && polyOut2.pts != null)
      {
        OutRec outRec1 = polyOut1 != polyOut2 ? (!this.Param1RightOfParam2(polyOut1, polyOut2) ? (!this.Param1RightOfParam2(polyOut2, polyOut1) ? this.GetLowermostRec(polyOut1, polyOut2) : polyOut1) : polyOut2) : polyOut1;
        OutPt p1;
        OutPt p2;
        if (this.JoinPoints(join1, out p1, out p2))
        {
          if (polyOut1 == polyOut2)
          {
            polyOut1.pts = this.GetBottomPt(p1);
            polyOut1.bottomPt = polyOut1.pts;
            polyOut1.bottomPt.idx = polyOut1.idx;
            OutRec outRec2 = this.CreateOutRec();
            this.m_PolyOuts.Add(outRec2);
            outRec2.idx = this.m_PolyOuts.Count - 1;
            join1.poly2Idx = outRec2.idx;
            outRec2.pts = this.GetBottomPt(p2);
            outRec2.bottomPt = outRec2.pts;
            outRec2.bottomPt.idx = outRec2.idx;
            if (this.Poly2ContainsPoly1(outRec2.pts, polyOut1.pts, this.m_UseFullRange))
            {
              outRec2.isHole = !polyOut1.isHole;
              outRec2.FirstLeft = polyOut1;
              this.FixupJoinRecs(join1, p2, index1 + 1);
              if (this.m_UsingPolyTree)
                this.FixupFirstLefts2(outRec2, polyOut1);
              this.FixupOutPolygon(polyOut1);
              this.FixupOutPolygon(outRec2);
              if ((outRec2.isHole ^ this.m_ReverseOutput) == this.Area(outRec2, this.m_UseFullRange) > 0.0)
                this.ReversePolyPtLinks(outRec2.pts);
            }
            else if (this.Poly2ContainsPoly1(polyOut1.pts, outRec2.pts, this.m_UseFullRange))
            {
              outRec2.isHole = polyOut1.isHole;
              polyOut1.isHole = !outRec2.isHole;
              outRec2.FirstLeft = polyOut1.FirstLeft;
              polyOut1.FirstLeft = outRec2;
              this.FixupJoinRecs(join1, p2, index1 + 1);
              if (this.m_UsingPolyTree)
                this.FixupFirstLefts2(polyOut1, outRec2);
              this.FixupOutPolygon(polyOut1);
              this.FixupOutPolygon(outRec2);
              if ((polyOut1.isHole ^ this.m_ReverseOutput) == this.Area(polyOut1, this.m_UseFullRange) > 0.0)
                this.ReversePolyPtLinks(polyOut1.pts);
            }
            else
            {
              outRec2.isHole = polyOut1.isHole;
              outRec2.FirstLeft = polyOut1.FirstLeft;
              this.FixupJoinRecs(join1, p2, index1 + 1);
              if (this.m_UsingPolyTree)
                this.FixupFirstLefts1(polyOut1, outRec2);
              this.FixupOutPolygon(polyOut1);
              this.FixupOutPolygon(outRec2);
            }
          }
          else
          {
            this.FixupOutPolygon(polyOut1);
            int idx1 = polyOut1.idx;
            int idx2 = polyOut2.idx;
            polyOut2.pts = (OutPt) null;
            polyOut2.bottomPt = (OutPt) null;
            polyOut1.isHole = outRec1.isHole;
            if (outRec1 == polyOut2)
              polyOut1.FirstLeft = polyOut2.FirstLeft;
            polyOut2.FirstLeft = polyOut1;
            for (int index2 = index1 + 1; index2 < this.m_Joins.Count; ++index2)
            {
              JoinRec join2 = this.m_Joins[index2];
              if (join2.poly1Idx == idx2)
                join2.poly1Idx = idx1;
              if (join2.poly2Idx == idx2)
                join2.poly2Idx = idx1;
            }
            if (this.m_UsingPolyTree)
              this.FixupFirstLefts2(polyOut2, polyOut1);
          }
        }
      }
    }
  }

  private static bool FullRangeNeeded(List<IntPoint> pts)
  {
    bool flag = false;
    for (int index = 0; index < pts.Count; ++index)
    {
      if (Math.Abs(pts[index].X) > 4611686018427387903L || Math.Abs(pts[index].Y) > 4611686018427387903L)
        throw new ClipperException("Coordinate exceeds range bounds.");
      if (Math.Abs(pts[index].X) > 1073741823L || Math.Abs(pts[index].Y) > 1073741823L)
        flag = true;
    }
    return flag;
  }

  public static double Area(List<IntPoint> poly)
  {
    int index1 = poly.Count - 1;
    if (index1 < 2)
      return 0.0;
    if (Clipper.FullRangeNeeded(poly))
    {
      Int128 int128_1 = new Int128(0L);
      Int128 int128_2 = Int128.Int128Mul(poly[index1].X + poly[0].X, poly[0].Y - poly[index1].Y);
      for (int index2 = 1; index2 <= index1; ++index2)
        int128_2 += Int128.Int128Mul(poly[index2 - 1].X + poly[index2].X, poly[index2].Y - poly[index2 - 1].Y);
      return int128_2.ToDouble() / 2.0;
    }
    double num = ((double) poly[index1].X + (double) poly[0].X) * ((double) poly[0].Y - (double) poly[index1].Y);
    for (int index3 = 1; index3 <= index1; ++index3)
      num += ((double) poly[index3 - 1].X + (double) poly[index3].X) * ((double) poly[index3].Y - (double) poly[index3 - 1].Y);
    return num / 2.0;
  }

  private double Area(OutRec outRec, bool UseFull64BitRange)
  {
    OutPt outPt = outRec.pts;
    if (outPt == null)
      return 0.0;
    if (UseFull64BitRange)
    {
      Int128 int128 = new Int128(0L);
      do
      {
        int128 += Int128.Int128Mul(outPt.pt.X + outPt.prev.pt.X, outPt.prev.pt.Y - outPt.pt.Y);
        outPt = outPt.next;
      }
      while (outPt != outRec.pts);
      return int128.ToDouble() / 2.0;
    }
    double num = 0.0;
    do
    {
      num += (double) ((outPt.pt.X + outPt.prev.pt.X) * (outPt.prev.pt.Y - outPt.pt.Y));
      outPt = outPt.next;
    }
    while (outPt != outRec.pts);
    return num / 2.0;
  }

  internal static List<IntPoint> BuildArc(IntPoint pt, double a1, double a2, double r)
  {
    long num1 = (long) Math.Max(6, (int) (Math.Sqrt(Math.Abs(r)) * Math.Abs(a2 - a1)));
    if (num1 > 256L)
      num1 = 256L;
    int capacity = (int) num1;
    List<IntPoint> intPointList = new List<IntPoint>(capacity);
    double num2 = (a2 - a1) / (double) (capacity - 1);
    double num3 = a1;
    for (int index = 0; index < capacity; ++index)
    {
      intPointList.Add(new IntPoint(pt.X + Clipper.Round(Math.Cos(num3) * r), pt.Y + Clipper.Round(Math.Sin(num3) * r)));
      num3 += num2;
    }
    return intPointList;
  }

  internal static Clipper.DoublePoint GetUnitNormal(IntPoint pt1, IntPoint pt2)
  {
    double num1 = (double) (pt2.X - pt1.X);
    double num2 = (double) (pt2.Y - pt1.Y);
    if (num1 == 0.0 && num2 == 0.0)
      return new Clipper.DoublePoint();
    double num3 = 1.0 / Math.Sqrt(num1 * num1 + num2 * num2);
    double num4 = num1 * num3;
    return new Clipper.DoublePoint(num2 * num3, -num4);
  }

  public static List<List<IntPoint>> OffsetPolygons(
    List<List<IntPoint>> poly,
    double delta,
    JoinType jointype,
    double MiterLimit,
    bool AutoFix)
  {
    List<List<IntPoint>> solution = new List<List<IntPoint>>(poly.Count);
    Clipper.PolyOffsetBuilder polyOffsetBuilder = new Clipper.PolyOffsetBuilder(poly, solution, delta, jointype, MiterLimit, AutoFix);
    return solution;
  }

  public static List<List<IntPoint>> OffsetPolygons(
    List<List<IntPoint>> poly,
    double delta,
    JoinType jointype,
    double MiterLimit)
  {
    List<List<IntPoint>> solution = new List<List<IntPoint>>(poly.Count);
    Clipper.PolyOffsetBuilder polyOffsetBuilder = new Clipper.PolyOffsetBuilder(poly, solution, delta, jointype, MiterLimit);
    return solution;
  }

  public static List<List<IntPoint>> OffsetPolygons(
    List<List<IntPoint>> poly,
    double delta,
    JoinType jointype)
  {
    List<List<IntPoint>> solution = new List<List<IntPoint>>(poly.Count);
    Clipper.PolyOffsetBuilder polyOffsetBuilder = new Clipper.PolyOffsetBuilder(poly, solution, delta, jointype);
    return solution;
  }

  public static List<List<IntPoint>> OffsetPolygons(List<List<IntPoint>> poly, double delta)
  {
    List<List<IntPoint>> solution = new List<List<IntPoint>>(poly.Count);
    Clipper.PolyOffsetBuilder polyOffsetBuilder = new Clipper.PolyOffsetBuilder(poly, solution, delta, JoinType.jtSquare);
    return solution;
  }

  public static List<List<IntPoint>> SimplifyPolygon(List<IntPoint> poly, PolyFillType fillType = PolyFillType.pftEvenOdd)
  {
    List<List<IntPoint>> solution = new List<List<IntPoint>>();
    Clipper clipper = new Clipper();
    clipper.AddPolygon(poly, PolyType.ptSubject);
    clipper.Execute(ClipType.ctUnion, solution, fillType, fillType);
    return solution;
  }

  public static List<List<IntPoint>> SimplifyPolygons(
    List<List<IntPoint>> polys,
    PolyFillType fillType = PolyFillType.pftEvenOdd)
  {
    List<List<IntPoint>> solution = new List<List<IntPoint>>();
    Clipper clipper = new Clipper();
    clipper.AddPolygons(polys, PolyType.ptSubject);
    clipper.Execute(ClipType.ctUnion, solution, fillType, fillType);
    return solution;
  }

  public static List<IntPoint> CleanPolygon(List<IntPoint> poly, double delta = 1.415)
  {
    int count = poly.Count;
    if (count < 3)
      return (List<IntPoint>) null;
    List<IntPoint> intPointList = new List<IntPoint>((IEnumerable<IntPoint>) poly);
    int num = (int) (delta * delta);
    IntPoint intPoint = poly[0];
    int index1 = 1;
    for (int index2 = 1; index2 < count; ++index2)
    {
      if ((poly[index2].X - intPoint.X) * (poly[index2].X - intPoint.X) + (poly[index2].Y - intPoint.Y) * (poly[index2].Y - intPoint.Y) > (long) num)
      {
        intPointList[index1] = poly[index2];
        intPoint = poly[index2];
        ++index1;
      }
    }
    intPoint = poly[index1 - 1];
    if ((poly[0].X - intPoint.X) * (poly[0].X - intPoint.X) + (poly[0].Y - intPoint.Y) * (poly[0].Y - intPoint.Y) <= (long) num)
      --index1;
    if (index1 < count)
      intPointList.RemoveRange(index1, count - index1);
    return intPointList;
  }

  public static void PolyTreeToPolygons(PolyTree polytree, List<List<IntPoint>> polygons)
  {
    polygons.Clear();
    polygons.Capacity = polytree.Total;
    Clipper.AddPolyNodeToPolygons((PolyNode) polytree, polygons);
  }

  public static void AddPolyNodeToPolygons(PolyNode polynode, List<List<IntPoint>> polygons)
  {
    if (polynode.Contour.Count > 0)
      polygons.Add(polynode.Contour);
    foreach (PolyNode child in polynode.Childs)
      Clipper.AddPolyNodeToPolygons(child, polygons);
  }

  internal class DoublePoint
  {
    public double X { get; set; }

    public double Y { get; set; }

    public DoublePoint(double x = 0.0, double y = 0.0)
    {
      this.X = x;
      this.Y = y;
    }
  }

  private class PolyOffsetBuilder
  {
    private const int buffLength = 128;
    private List<List<IntPoint>> pts;
    private List<IntPoint> currentPoly;
    private List<Clipper.DoublePoint> normals;
    private double delta;
    private double m_R;
    private int m_i;
    private int m_j;
    private int m_k;

    public PolyOffsetBuilder(
      List<List<IntPoint>> pts,
      List<List<IntPoint>> solution,
      double delta,
      JoinType jointype,
      double MiterLimit = 2.0,
      bool AutoFix = true)
    {
      if (delta == 0.0)
      {
        solution = pts;
      }
      else
      {
        this.pts = pts;
        this.delta = delta;
        if (AutoFix)
        {
          int count = pts.Count;
          int index1 = 0;
          while (index1 < count && pts[index1].Count == 0)
            ++index1;
          if (index1 == count)
            return;
          IntPoint botPt = pts[index1][0];
          for (int index2 = index1; index2 < count; ++index2)
          {
            if (pts[index2].Count != 0)
            {
              if (this.UpdateBotPt(pts[index2][0], ref botPt))
                index1 = index2;
              for (int index3 = pts[index2].Count - 1; index3 > 0; --index3)
              {
                if (ClipperBase.PointsEqual(pts[index2][index3], pts[index2][index3 - 1]))
                  pts[index2].RemoveAt(index3);
                else if (this.UpdateBotPt(pts[index2][index3], ref botPt))
                  index1 = index2;
              }
            }
          }
          if (!Clipper.Orientation(pts[index1]))
            Clipper.ReversePolygons(pts);
        }
        if (MiterLimit <= 1.0)
          MiterLimit = 1.0;
        double num = 2.0 / (MiterLimit * MiterLimit);
        this.normals = new List<Clipper.DoublePoint>();
        solution.Clear();
        solution.Capacity = pts.Count;
        for (this.m_i = 0; this.m_i < pts.Count; ++this.m_i)
        {
          int count = pts[this.m_i].Count;
          if (count > 1 && pts[this.m_i][0].X == pts[this.m_i][count - 1].X && pts[this.m_i][0].Y == pts[this.m_i][count - 1].Y)
            --count;
          if (count != 0 && (count >= 3 || delta > 0.0))
          {
            if (count == 1)
            {
              List<IntPoint> intPointList = Clipper.BuildArc(pts[this.m_i][count - 1], 0.0, 2.0 * Math.PI, delta);
              solution.Add(intPointList);
            }
            else
            {
              this.normals.Clear();
              this.normals.Capacity = count;
              for (int index = 0; index < count - 1; ++index)
                this.normals.Add(Clipper.GetUnitNormal(pts[this.m_i][index], pts[this.m_i][index + 1]));
              this.normals.Add(Clipper.GetUnitNormal(pts[this.m_i][count - 1], pts[this.m_i][0]));
              this.currentPoly = new List<IntPoint>();
              this.m_k = count - 1;
              for (this.m_j = 0; this.m_j < count; ++this.m_j)
              {
                switch (jointype)
                {
                  case JoinType.jtSquare:
                    this.DoSquare(1.0);
                    break;
                  case JoinType.jtRound:
                    this.DoRound();
                    break;
                  case JoinType.jtMiter:
                    this.m_R = 1.0 + (this.normals[this.m_j].X * this.normals[this.m_k].X + this.normals[this.m_j].Y * this.normals[this.m_k].Y);
                    if (this.m_R >= num)
                    {
                      this.DoMiter();
                      break;
                    }
                    this.DoSquare(MiterLimit);
                    break;
                }
                this.m_k = this.m_j;
              }
              solution.Add(this.currentPoly);
            }
          }
        }
        Clipper clipper = new Clipper();
        clipper.AddPolygons(solution, PolyType.ptSubject);
        if (delta > 0.0)
        {
          clipper.Execute(ClipType.ctUnion, solution, PolyFillType.pftPositive, PolyFillType.pftPositive);
        }
        else
        {
          IntRect bounds = clipper.GetBounds();
          clipper.AddPolygon(new List<IntPoint>(4)
          {
            new IntPoint(bounds.left - 10L, bounds.bottom + 10L),
            new IntPoint(bounds.right + 10L, bounds.bottom + 10L),
            new IntPoint(bounds.right + 10L, bounds.top - 10L),
            new IntPoint(bounds.left - 10L, bounds.top - 10L)
          }, PolyType.ptSubject);
          clipper.Execute(ClipType.ctUnion, solution, PolyFillType.pftNegative, PolyFillType.pftNegative);
          if (solution.Count <= 0)
            return;
          solution.RemoveAt(0);
          for (int index = 0; index < solution.Count; ++index)
            solution[index].Reverse();
        }
      }
    }

    internal bool UpdateBotPt(IntPoint pt, ref IntPoint botPt)
    {
      if (pt.Y <= botPt.Y && (pt.Y != botPt.Y || pt.X >= botPt.X))
        return false;
      botPt = pt;
      return true;
    }

    internal void AddPoint(IntPoint pt)
    {
      int count = this.currentPoly.Count;
      if (count == this.currentPoly.Capacity)
        this.currentPoly.Capacity = count + 128;
      this.currentPoly.Add(pt);
    }

    internal void DoSquare(double mul)
    {
      IntPoint pt1 = new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + this.normals[this.m_k].X * this.delta), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + this.normals[this.m_k].Y * this.delta));
      IntPoint pt2 = new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + this.normals[this.m_j].X * this.delta), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + this.normals[this.m_j].Y * this.delta));
      if ((this.normals[this.m_k].X * this.normals[this.m_j].Y - this.normals[this.m_j].X * this.normals[this.m_k].Y) * this.delta >= 0.0)
      {
        double num1 = Math.Atan2(this.normals[this.m_k].Y, this.normals[this.m_k].X);
        double num2 = Math.Abs(Math.Atan2(-this.normals[this.m_j].Y, -this.normals[this.m_j].X) - num1);
        if (num2 > Math.PI)
          num2 = 2.0 * Math.PI - num2;
        double num3 = Math.Tan((Math.PI - num2) / 4.0) * Math.Abs(this.delta * mul);
        pt1 = new IntPoint((long) ((double) pt1.X - this.normals[this.m_k].Y * num3), (long) ((double) pt1.Y + this.normals[this.m_k].X * num3));
        this.AddPoint(pt1);
        pt2 = new IntPoint((long) ((double) pt2.X + this.normals[this.m_j].Y * num3), (long) ((double) pt2.Y - this.normals[this.m_j].X * num3));
        this.AddPoint(pt2);
      }
      else
      {
        this.AddPoint(pt1);
        this.AddPoint(this.pts[this.m_i][this.m_j]);
        this.AddPoint(pt2);
      }
    }

    internal void DoMiter()
    {
      if ((this.normals[this.m_k].X * this.normals[this.m_j].Y - this.normals[this.m_j].X * this.normals[this.m_k].Y) * this.delta >= 0.0)
      {
        double num = this.delta / this.m_R;
        this.AddPoint(new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + (this.normals[this.m_k].X + this.normals[this.m_j].X) * num), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + (this.normals[this.m_k].Y + this.normals[this.m_j].Y) * num)));
      }
      else
      {
        IntPoint pt1 = new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + this.normals[this.m_k].X * this.delta), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + this.normals[this.m_k].Y * this.delta));
        IntPoint pt2 = new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + this.normals[this.m_j].X * this.delta), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + this.normals[this.m_j].Y * this.delta));
        this.AddPoint(pt1);
        this.AddPoint(this.pts[this.m_i][this.m_j]);
        this.AddPoint(pt2);
      }
    }

    internal void DoRound()
    {
      IntPoint pt1 = new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + this.normals[this.m_k].X * this.delta), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + this.normals[this.m_k].Y * this.delta));
      IntPoint pt2 = new IntPoint(Clipper.Round((double) this.pts[this.m_i][this.m_j].X + this.normals[this.m_j].X * this.delta), Clipper.Round((double) this.pts[this.m_i][this.m_j].Y + this.normals[this.m_j].Y * this.delta));
      this.AddPoint(pt1);
      if ((this.normals[this.m_k].X * this.normals[this.m_j].Y - this.normals[this.m_j].X * this.normals[this.m_k].Y) * this.delta >= 0.0)
      {
        if (this.normals[this.m_j].X * this.normals[this.m_k].X + this.normals[this.m_j].Y * this.normals[this.m_k].Y < 0.985)
        {
          double a1 = Math.Atan2(this.normals[this.m_k].Y, this.normals[this.m_k].X);
          double a2 = Math.Atan2(this.normals[this.m_j].Y, this.normals[this.m_j].X);
          if (this.delta > 0.0 && a2 < a1)
            a2 += 2.0 * Math.PI;
          else if (this.delta < 0.0 && a2 > a1)
            a2 -= 2.0 * Math.PI;
          List<IntPoint> intPointList = Clipper.BuildArc(this.pts[this.m_i][this.m_j], a1, a2, this.delta);
          for (int index = 0; index < intPointList.Count; ++index)
            this.AddPoint(intPointList[index]);
        }
      }
      else
        this.AddPoint(this.pts[this.m_i][this.m_j]);
      this.AddPoint(pt2);
    }
  }
}