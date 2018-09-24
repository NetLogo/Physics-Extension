enablePlugins(org.nlogo.build.NetLogoExtension)

netLogoExtName      := "phys"

netLogoClassManager := "PhysExtension"

netLogoZipSources   := false

scalaVersion           := "2.12.6"

scalaSource in Compile := baseDirectory.value / "src"

scalacOptions          ++= Seq("-deprecation", "-unchecked", "-Xfatal-warnings", "-encoding", "us-ascii")

// The remainder of this file is for options specific to bundled netlogo extensions
// if copying this extension to build your own, you need nothing past line 14 to build
// sample-scala.zip
netLogoTarget :=
  org.nlogo.build.NetLogoExtension.directoryTarget(baseDirectory.value)

libraryDependencies ++= Seq(
	"org.dyn4j" % "dyn4j" % "3.2.4"
)

netLogoVersion := "6.0.4"
