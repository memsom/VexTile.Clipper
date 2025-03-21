namespace VexTile.ClipperLib;

[Flags]
internal enum Protects
{
  ipNone = 0,
  ipLeft = 1,
  ipRight = 2,
  ipBoth = ipRight | ipLeft, // 0x00000003
}