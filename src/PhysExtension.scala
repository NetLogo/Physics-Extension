import org.nlogo.{agent, api, core, nvm}
import core.Syntax
import api.ScalaConversions._
import api.{Argument, Context, ExtensionManager}
import agent.{Patch, Turtle, World}
import org.nlogo.core.AgentKind
import java.lang.StrictMath

import org.dyn4j.dynamics
import org.dyn4j.dynamics.{Body, Force}
import org.dyn4j.geometry.{Geometry, Mass, MassType, Vector2}

import scala.collection.mutable

class PhysExtension extends api.DefaultClassManager {
  var world: dynamics.World = new dynamics.World()
  var turtleChanges: Integer = 0
  var turtlesToBodies: mutable.Map[Turtle, Body] = mutable.LinkedHashMap[Turtle, Body]()
  var patchesToBodies: mutable.Map[Patch, Body] = mutable.LinkedHashMap[Patch, Body]()

  def load(manager: api.PrimitiveManager) = {
    manager.addPrimitive("set-physical", SetPhysical)
    manager.addPrimitive("set-gravity", SetGravity)
    manager.addPrimitive("update", Update)
    manager.addPrimitive("forward", Forward)
    manager.addPrimitive("total-ke",TotalKE)
    manager.addPrimitive("stop-all",StopAll)
    manager.addPrimitive("changes",  NumChanges)

  }

  override def runOnce(em: ExtensionManager): Unit = {
    super.runOnce(em)
    clearAll()
  }

  override def clearAll(): Unit = {
    super.clearAll()
    world = new dynamics.World()
    world.getSettings.setRestitutionVelocity(0)
    world.setGravity(new Vector2(0,0))
    turtlesToBodies.clear()
    patchesToBodies.clear()
    turtleChanges = 0
  }

  object SetPhysical extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.BooleanType), agentClassString = "-TP-")

    override def perform(args: Array[Argument], context: Context): Unit = {
      context.getAgent match {
        case turtle: Turtle =>
          if (args(0).getBoolean) {
            turtlesToBodies.getOrElseUpdate(turtle, {
              val body = new Body()
              val fixture = body.addFixture(Geometry.createCircle(turtle.size / 2))
              fixture.setRestitution(1)
              fixture.setFriction(0)
              body.getTransform.setTranslation(turtle.xcor, turtle.ycor)
              body.getTransform.setRotation(StrictMath.toRadians(90 - turtle.heading))
              body.setMass(MassType.NORMAL)
              body.setAngularDamping(0)
              body.setLinearDamping(0)
              body.setAutoSleepingEnabled(false)

              world.addBody(body)
              body
            })

          }
          else {
            turtlesToBodies.remove(turtle).foreach(b => world.removeBody(b))

          }
        case patch: Patch =>
          if (args(0).getBoolean) {
            patchesToBodies.getOrElseUpdate(patch, {
              val body = new Body
              val fixture = body.addFixture(Geometry.createSquare(1))
              fixture.setFriction(0)
              fixture.setRestitution(1)
              body.setMass(MassType.INFINITE)
              body.getTransform.setTranslation(patch.pxcor, patch.pycor)
              world.addBody(body)
              body
            })
          } else {
            patchesToBodies.remove(patch).foreach(b => world.removeBody(b))
          }
      }
    }
  }

  object Update extends api.Command {
    override def perform(args: Array[Argument], context: Context): Unit = {

      turtlesToBodies.foreach { case (turtle, body) =>
        if (turtle.id < 0) {
          world.removeBody(body)
          turtlesToBodies.remove(turtle)
        } else {
          if (turtle.xcor != body.getWorldCenter.x || turtle.ycor != body.getWorldCenter.y) {
            turtleChanges += 1
            body.getTransform.setTranslation(turtle.xcor, turtle.ycor)
          }
          val radians = StrictMath.toRadians(90 - turtle.heading)
          if (radians != body.getTransform.getRotation) {
            body.getTransform.setRotation(radians)

          }

          // TODO: Set Size
        }
      }
      world.update(args(0).getDoubleValue, 60)
      turtlesToBodies.foreach { case (turtle, body) =>

        if (turtle.xcor != body.getWorldCenter.x || turtle.ycor != body.getWorldCenter.y) {
          turtle.xcor(body.getWorldCenter.x)
          turtle.ycor(body.getWorldCenter.y)

        }
        val degrees = 90 - StrictMath.toDegrees(body.getTransform.getRotation)
        if (degrees != turtle.heading) {
          turtle.heading(degrees)

        }

        // TODO: Set Size
      }


    }

    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType), agentClassString = "O---")
  }

  object SetGravity extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType, Syntax.NumberType))

    override def perform(args: Array[Argument], context: Context): Unit =
      world.setGravity(new Vector2(args(0).getDoubleValue, args(1).getDoubleValue))
  }

  object Forward extends api.Command
  {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType), agentClassString = "-T--")

    override def perform(args: Array[Argument], context: Context): Unit = {

      val turtle = context.getAgent.asInstanceOf[Turtle]
      val amount = args(0).getDoubleValue
      turtlesToBodies(turtle).applyForce(new Force(turtle.dx * amount, turtle.dy * amount))


    }


  }
  object TotalKE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      var total : Double = 0.0
      turtlesToBodies.foreach { case (turtle, body) => total += body.getMass.getMass * body.getLinearVelocity.getMagnitude * body.getLinearVelocity.getMagnitude}

      Double.box(total)

    }


  }
  object NumChanges extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {

      Int.box(turtleChanges)
    }


  }

  object StopAll extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List())

    override def perform(args: Array[Argument], context: Context): Unit = {
      turtlesToBodies.foreach { case (turtle, body) => body.setLinearVelocity(new Vector2(0.0, 0.0))}
      }
  }
}

