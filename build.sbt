enablePlugins(org.nlogo.build.NetLogoExtension)

name := "Physics-Extension"
version := "0.4.2"

netLogoExtName      := "phys"
netLogoClassManager := "org.nlogo.extensions.phys.PhysicsExtension"
netLogoVersion      := "6.3.0"
netLogoZipExtras   ++= Seq(baseDirectory.value / "demos")

scalaVersion          := "2.12.6"
Compile / scalaSource := baseDirectory.value / "src" / "main"
Test / scalaSource    := baseDirectory.value / "src" / "test"
scalacOptions        ++= Seq("-deprecation", "-unchecked", "-Xfatal-warnings", "-encoding", "us-ascii", "-release", "11")

libraryDependencies ++= Seq(
  "org.dyn4j" % "dyn4j" % "3.2.4"
)
