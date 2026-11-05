enablePlugins(org.nlogo.build.NetLogoExtension)

name       := "Physics-Extension"
version    := "0.4.4"
isSnapshot := true

scalaVersion           := "3.7.0"
Compile / scalaSource  := baseDirectory.value / "src" / "main"
Test / scalaSource     := baseDirectory.value / "src" / "test"
scalacOptions          ++= Seq("-deprecation", "-unchecked", "-Xfatal-warnings", "-encoding", "us-ascii",
                               "-release", "21", "-Wunused:linted")

libraryDependencies ++= Seq(
  "org.dyn4j" % "dyn4j" % "3.2.4"
)

netLogoExtName      := "phys"
netLogoClassManager := "org.nlogo.extensions.phys.PhysicsExtension"
netLogoVersion      := "7.1.0-alpha1-931b371"
netLogoZipExtras    ++= Seq(baseDirectory.value / "demos")
