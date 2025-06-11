package org.nlogo.extensions.phys

import org.nlogo.{ agent, api, core, nvm }
import core.Syntax
import api.ScalaConversions._
import api.{Argument, Context, ExtensionManager, ScalaConversions}
import agent._
import org.nlogo.core.AgentKind
import java.lang.StrictMath
import org.nlogo.api.LogoListBuilder

import org.dyn4j.collision.narrowphase.Penetration
import org.dyn4j.dynamics
import org.dyn4j.dynamics._
import org.dyn4j.dynamics.contact.{ContactAdapter, ContactPoint, SolvedContactPoint}
import org.dyn4j.dynamics.contact.ContactConstraint
import org.dyn4j.geometry.{Geometry, Mass, MassType, Vector2}

import scala.collection.mutable
import scala.collection.mutable.ListBuffer

class PhysicsExtension extends api.DefaultClassManager {
  var world: dynamics.World = new dynamics.World()
  var turtleChanges: Double = 10
  var collisions: Double = 0
  var patchesCollisionList: mutable.Map[Turtle, List[Patch]] = mutable.LinkedHashMap[Turtle, List[Patch]]()
  var turtlesCollisionList: mutable.Map[Turtle, List[Turtle]] = mutable.LinkedHashMap[Turtle, List[Turtle]]()
  var turtlesCollisionListPatches: mutable.Map[Patch, List[Turtle]] = mutable.LinkedHashMap[Patch, List[Turtle]]()
  var turtlesToBodies: mutable.Map[Turtle, Body] = mutable.LinkedHashMap[Turtle, Body]()
  var patchesToBodies: mutable.Map[Patch, Body] = mutable.LinkedHashMap[Patch, Body]()
  var bodiesToTurtles: mutable.Map[Body, Turtle] = mutable.LinkedHashMap[Body, Turtle]()
  var bodiesToPatches: mutable.Map[Body, Patch] = mutable.LinkedHashMap[Body, Patch]()
  var turtlesLastE: mutable.Map[Turtle, Vector2] = mutable.LinkedHashMap[Turtle, Vector2]()
  var turtlesLastV: mutable.Map[Turtle, Vector2] = mutable.LinkedHashMap[Turtle, Vector2]()
  var collisionSetList: mutable.ListBuffer[mutable.Set[Body]] = mutable.ListBuffer[mutable.Set[Body]]()
  var cTime: Long = 0
  var floor: Double = -16.0
  var lastGrav: Vector2 = new Vector2(0.0, 0.0)
  var newE: Double = 0.0
  var newEa: Double = 0.0
  var outOfOrder: Long = 0
  var eDiffTolerance = 0.00000000000001
  var numCorrections: Long = 0
  var numUncorrectable: Long = 0
  var numEnergyDiscrepancies: Long = 0
  var totalEnergyDiscrepancy: Double = 0
  var doConservation: Boolean = true
  var doCollisionDetection: Boolean = true


  def load(manager: api.PrimitiveManager)  = {
    manager.addPrimitive("set-physical", SetPhysical)
    manager.addPrimitive("set-gravity", SetGravity)
    manager.addPrimitive("update", Update)
    manager.addPrimitive("push", Forward)
    manager.addPrimitive("apply-force", ApplyForce)
    manager.addPrimitive("get-total-corrections", TotalCorrections)
    manager.addPrimitive("get-total-uncorrectable", TotalUncorrectable)
    manager.addPrimitive("get-total-energy-discrepancy", EnergyDiscrepancy)
    manager.addPrimitive("get-num-energy-discrepancies", EnergyDiscrepancies)
    manager.addPrimitive("total-ke", TotalKE)
    manager.addPrimitive("total-e", TotalE)
    manager.addPrimitive("total-newe", TotalNewE)
    manager.addPrimitive("total-laste", TotalLastE)
    manager.addPrimitive("get-v", GetV)
    manager.addPrimitive("set-v-magnitude", SetVMag)
    manager.addPrimitive("get-ke", GetKE)
    manager.addPrimitive("get-e", GetE)
    manager.addPrimitive("get-vx", GetVx)
    manager.addPrimitive("get-vy", GetVy)
    manager.addPrimitive("get-mass", GetMass)
    manager.addPrimitive("get-center-of-mass", CenterOfMass)
    manager.addPrimitive("get-inertia", Inertia)
    manager.addPrimitive("set-mass", SetMass)
    manager.addPrimitive("set-v", SetV)
    manager.addPrimitive("stop-all", StopAll)
    manager.addPrimitive("changes",  NumChanges)
    manager.addPrimitive("collision-number",  ColNumber)
    manager.addPrimitive("get-turtle-collisions", TurtCols)
    manager.addPrimitive("get-patch-collisions", PatchCols)
    manager.addPrimitive("get-mlc", GetMLC)
    manager.addPrimitive("do-conservation", SetConservation)
    manager.addPrimitive("do-collision-detection", SetCollisionDetection)
  }

  override def runOnce(em: ExtensionManager): Unit = {
    super.runOnce(em)
    clearAll()
  }

  override def clearAll(): Unit = {
    super.clearAll()
    world = new dynamics.World()
    world.getSettings.setRestitutionVelocity(0)
    world.getSettings.setContinuousDetectionMode(ContinuousDetectionMode.ALL)
    world.setGravity(new Vector2(0,0))
    world.getSettings.setVelocityConstraintSolverIterations(1)
    world.getSettings.setPositionConstraintSolverIterations(1)
    turtlesToBodies.clear()
    patchesToBodies.clear()
    bodiesToTurtles.clear()
    bodiesToPatches.clear()
    turtlesLastV.clear()
    turtlesLastE.clear()
    turtleChanges = 0
    numCorrections = 0
    numUncorrectable = 0
    collisions = 0
    outOfOrder = 0
    numEnergyDiscrepancies = 0
    totalEnergyDiscrepancy = 0.0
    lastGrav = new Vector2(0.0, 0.0)
    newE = 0.0
    newEa = 0.0
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
              fixture.setRestitution(1.0)
              fixture.setFriction(0)
              body.getTransform.setTranslation(turtle.xcor, turtle.ycor)
              body.getTransform.setRotation(StrictMath.toRadians(90 - turtle.heading))
              body.setMass(MassType.NORMAL)
              body.setAngularDamping(0)
              body.setLinearDamping(0)
              body.setAutoSleepingEnabled(false)
              // turtlesLastE.getOrElseUpdate(turtle, new Vector2((floor - turtle.ycor) * world.getGravity.getYComponent.y * body.getMass.getMass, 0.0))
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
              fixture.setRestitution(1.0)
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
          //turtlesLastE.remove(turtle)
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
      bodiesToTurtles = turtlesToBodies.map(_.swap)
      bodiesToPatches = patchesToBodies.map(_.swap)
      patchesCollisionList = mutable.LinkedHashMap[Turtle, List[Patch]]()
      turtlesCollisionList = mutable.LinkedHashMap[Turtle, List[Turtle]]()
      turtlesCollisionListPatches = mutable.LinkedHashMap[Patch, List[Turtle]]()
      newEa = 0.0
      if (doCollisionDetection)
      world.addListener(new ContactHandler())
      world.update(args(0).getDoubleValue, 60)
      world.removeAllListeners()
      if (StrictMath.abs(totalEnergyDiscrepancy) > eDiffTolerance)
      {
        turtlesToBodies.foreach { case (turtle, body) =>
          if (StrictMath.abs(totalEnergyDiscrepancy) > eDiffTolerance) {
            val ke: Double = 0.5 * body.getMass.getMass * StrictMath.pow(body.getLinearVelocity.getMagnitude, 2.0)
            var eDiff: Double = 0.0
            if (totalEnergyDiscrepancy < (ke / 20.0)) {
              eDiff = totalEnergyDiscrepancy
            }
            else {
              eDiff = (ke / 20.0)
            }
            val vFin: Double = StrictMath.sqrt(StrictMath.pow(body.getLinearVelocity.getMagnitude, 2.0) - (2 * eDiff / body.getMass.getMass))
            body.getLinearVelocity.setMagnitude(vFin)
            totalEnergyDiscrepancy -= eDiff
          }
        }
      }

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
      turtlesLastE.clear()
    }

    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType), agentClassString = "O---")
  }

  object SetConservation extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.BooleanType), agentClassString = "O---")
    override def perform(args: Array[Argument], context: Context): Unit = {
       if (args(0).getBoolean) {doConservation = true } else {doConservation = false}
    }

  }

  object SetCollisionDetection extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.BooleanType), agentClassString = "O---")
    override def perform(args: Array[Argument], context: Context): Unit = {
      if (args(0).getBoolean) {doCollisionDetection = true } else {doCollisionDetection = false}
    }

  }


  object SetGravity extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType, Syntax.NumberType))

    override def perform(args: Array[Argument], context: Context): Unit = {
      if (new Vector2(args(0).getDoubleValue, args(1).getDoubleValue) != lastGrav) {
        world.setGravity(new Vector2(args(0).getDoubleValue, args(1).getDoubleValue))
      //  turtlesToBodies.foreach { case (turtle, body) => turtlesLastE(turtle).set((floor - turtle.ycor) * world.getGravity.getYComponent.y * body.getMass.getMass, turtlesLastE(turtle).getYComponent.y) }
      //  lastGrav = new Vector2(world.getGravity)
      }
    }

  }

  object Forward extends api.Command
  {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType), agentClassString = "-T--")

    override def perform(args: Array[Argument], context: Context): Unit = {

      val turtle = context.getAgent.asInstanceOf[Turtle]
      val amount = args(0).getDoubleValue
      turtlesToBodies(turtle).applyForce(new Force(turtle.dx * amount, turtle.dy * amount))
      //turtlesLastV.getOrElseUpdate(turtle, new Vector2(0.0, 0.0)).set(turtlesToBodies(turtle).getLinearVelocity)
      //turtlesLastE(turtle).set((floor - turtlesToBodies(turtle).getTransform.getTranslationY) * world.getGravity.y * turtlesToBodies(turtle).getMass.getMass, 0.5 * turtlesToBodies(turtle).getMass.getMass * StrictMath.pow(turtlesToBodies(turtle).getLinearVelocity.getMagnitude, 2.0) )

    }


  }


  object ApplyForce extends api.Command
  {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType, Syntax.NumberType), agentClassString = "-T--")

    override def perform(args: Array[Argument], context: Context): Unit = {

      val turtle = context.getAgent.asInstanceOf[Turtle]
      val amount = args(0).getDoubleValue
      val heading = ((90 - args(1).getDoubleValue) / 360.0) * 2 * StrictMath.PI

      turtlesToBodies(turtle).applyForce(new Force(StrictMath.cos(heading) * amount, StrictMath.sin(heading) * amount))
      //turtlesLastV.getOrElseUpdate(turtle, new Vector2(0.0, 0.0)).set(turtlesToBodies(turtle).getLinearVelocity)
      //turtlesLastE(turtle).set((floor - turtlesToBodies(turtle).getTransform.getTranslationY) * world.getGravity.y * turtlesToBodies(turtle).getMass.getMass, 0.5 * turtlesToBodies(turtle).getMass.getMass * StrictMath.pow(turtlesToBodies(turtle).getLinearVelocity.getMagnitude, 2.0) )
    }


  }


  object TotalKE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      var total : Double = 0.0
      turtlesToBodies.foreach { case (turtle, body) => total += 0.5 * body.getMass.getMass * body.getLinearVelocity.getMagnitude * body.getLinearVelocity.getMagnitude}

      Double.box(total)

    }


  }

  object TotalE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      var total : Double = 0.0
      turtlesToBodies.foreach { case (turtle, body) => {
        total += 0.5 * body.getMass.getMass * body.getLinearVelocity.getMagnitude * body.getLinearVelocity.getMagnitude
        total += (floor - body.getTransform.getTranslationY) * world.getGravity.getYComponent.y * body.getMass.getMass
      }}
      Double.box(total)
    }


  }

  object TotalLastE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {
     /* var total : Double = 0.0
      turtlesToBodies.foreach { case (turtle, body) => {
        total += turtlesLastE(turtle).x + turtlesLastE(turtle).y
      }} */
      Double.box(0.0)
    }


  }



  // This is where collision conservation should be happening!! //
  class ContactHandler extends ContactAdapter {

    override def preSolve(point: ContactPoint): Boolean = {

      if (bodiesToTurtles.contains(point.getBody1) && bodiesToTurtles.contains(point.getBody2)) { //Store energy for Bodies before Contact is solved.
        val turtle1: Turtle = bodiesToTurtles(point.getBody1)
        val turtle2: Turtle = bodiesToTurtles(point.getBody2)
        val ke1: Double = 0.5 * point.getBody1.getMass.getMass * StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0)
        val ke2: Double = 0.5 * point.getBody2.getMass.getMass * StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0)
        val pe1: Double = (floor - point.getBody1.getTransform.getTranslationY) * world.getGravity.y * point.getBody1.getMass.getMass
        val pe2: Double = (floor - point.getBody2.getTransform.getTranslationY) * world.getGravity.y * point.getBody2.getMass.getMass
        turtlesLastE.put(turtle1, new Vector2(pe1, ke1))
        turtlesLastE.put(turtle2, new Vector2(pe2, ke2))
      }
      else if (bodiesToTurtles.contains(point.getBody1) && bodiesToPatches.contains(point.getBody2)) {
        val turtle1: Turtle = bodiesToTurtles(point.getBody1)
        val ke1: Double = 0.5 * point.getBody1.getMass.getMass * StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0)
        val pe1: Double = (floor - point.getBody1.getTransform.getTranslationY) * world.getGravity.y * point.getBody1.getMass.getMass
        turtlesLastE.put(turtle1, new Vector2(pe1, ke1))
      }
      else if (bodiesToPatches.contains(point.getBody1) && bodiesToTurtles.contains(point.getBody2)) {
        val turtle2: Turtle = bodiesToTurtles(point.getBody2)
        val pe2: Double = (floor - point.getBody2.getTransform.getTranslationY) * world.getGravity.y * point.getBody2.getMass.getMass
        val ke2: Double = 0.5 * point.getBody2.getMass.getMass * StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0)
        turtlesLastE.put(turtle2 , new Vector2(pe2, ke2))
      }
      super.preSolve(point)
    }

    override def postSolve(point: SolvedContactPoint): Unit = {
      if (bodiesToTurtles.contains(point.getBody1) && bodiesToTurtles.contains(point.getBody2)) {
        val turtle1: Turtle = bodiesToTurtles(point.getBody1)
        val turtle2: Turtle = bodiesToTurtles(point.getBody2)
        val ke1: Double = 0.5 * point.getBody1.getMass.getMass * StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0)
        val ke2: Double = 0.5 * point.getBody2.getMass.getMass * StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0)
        val pe1: Double = (floor - point.getBody1.getTransform.getTranslationY) * world.getGravity.y * point.getBody1.getMass.getMass
        val pe2: Double = (floor - point.getBody2.getTransform.getTranslationY) * world.getGravity.y * point.getBody2.getMass.getMass
        val oldE: Double = turtlesLastE(turtle1).x + turtlesLastE(turtle1).y + turtlesLastE(turtle2).x + turtlesLastE(turtle2).y
        val newE: Double = ke1 + ke2 + pe1 + pe2
        val eDiff: Double = newE - oldE
         //Old Energy Differentials:
         //val eDiff1: Double = eDiff * point.getBody1.getMass.getMass / (point.getBody2.getMass.getMass + point.getBody1.getMass.getMass)
         //val eDiff2: Double = eDiff * point.getBody2.getMass.getMass / (point.getBody2.getMass.getMass + point.getBody1.getMass.getMass)

        val eDiff1: Double = eDiff * (ke1 + pe1) / (ke1 + pe1 + ke2 + pe2)
        val eDiff2: Double = eDiff * (ke2 + pe2) / (ke1 + pe1 + ke2 + pe2)
        if (eDiff > newE) {numUncorrectable += 1}

        val vFin1: Double = StrictMath.sqrt(StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0) - (2 * eDiff1 / point.getBody1.getMass.getMass))
        val vFin2: Double = StrictMath.sqrt(StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0) - (2 * eDiff2 / point.getBody2.getMass.getMass))


        if (vFin1 >= 0 && vFin2 >= 0) {
          if ((StrictMath.abs(eDiff) > eDiffTolerance) && doConservation) {
            numCorrections += 1
            point.getBody1.getLinearVelocity.setMagnitude(vFin1)
            point.getBody2.getLinearVelocity.setMagnitude(vFin2)
          }
        }
        else {numUncorrectable += 1 }

        val kef1: Double = 0.5 * point.getBody1.getMass.getMass * StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0)
        val kef2: Double = 0.5 * point.getBody2.getMass.getMass * StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0)
        val pef1: Double = (floor - point.getBody1.getTransform.getTranslationY) * world.getGravity.y * point.getBody1.getMass.getMass
        val pef2: Double = (floor - point.getBody2.getTransform.getTranslationY) * world.getGravity.y * point.getBody2.getMass.getMass
        turtlesLastE.put(turtle1, new Vector2(pef1, kef1))
        turtlesLastE.put(turtle2, new Vector2(pef2, kef2))
        val newEf: Double = kef1 + kef2 + pef1 + pef2
        val eDiffF: Double = newEf - oldE
        if (StrictMath.abs(eDiffF) > eDiffTolerance)
          {
            totalEnergyDiscrepancy += newEf - oldE
            numEnergyDiscrepancies += 1
          }

      }
      else if (bodiesToTurtles.contains(point.getBody1) && bodiesToPatches.contains(point.getBody2)) {
        val turtle = bodiesToTurtles(point.getBody1)
        val ke1: Double = 0.5 * point.getBody1.getMass.getMass * StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0)
        val pe1: Double = (floor - point.getBody1.getTransform.getTranslationY) * world.getGravity.y * point.getBody1.getMass.getMass
        val oldE: Double = turtlesLastE(turtle).x + turtlesLastE(turtle).y
        val newE: Double = ke1 + pe1
        val eDiff: Double = newE - oldE
        val vFin: Double = StrictMath.sqrt(StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0) - (2 * eDiff / point.getBody1.getMass.getMass))
        if (vFin >= 0) {
          if ((StrictMath.abs(eDiff) > eDiffTolerance) && doConservation) {
            numCorrections += 1
            point.getBody1.getLinearVelocity.setMagnitude(vFin)
          }
        }
        val kef1: Double = 0.5 * point.getBody1.getMass.getMass * StrictMath.pow(point.getBody1.getLinearVelocity.getMagnitude, 2.0)
        val pef1: Double = (floor - point.getBody1.getTransform.getTranslationY) * world.getGravity.y * point.getBody1.getMass.getMass
        turtlesLastE.put(turtle, new Vector2(pef1, kef1))
        val newEf: Double = kef1 + pef1
        val eDiffF: Double = newEf - oldE
        if (StrictMath.abs(eDiffF) > eDiffTolerance)
        {
          totalEnergyDiscrepancy += newEf - oldE
          numEnergyDiscrepancies += 1
         // numWallDiscrepancies += 1
        }

      }
      else if (bodiesToPatches.contains(point.getBody1) && bodiesToTurtles.contains(point.getBody2)) {
        val turtle = bodiesToTurtles(point.getBody2)
        val ke2: Double = 0.5 * point.getBody2.getMass.getMass * StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0)
        val pe2: Double = (floor - point.getBody2.getTransform.getTranslationY) * world.getGravity.y * point.getBody2.getMass.getMass
        val oldE: Double = turtlesLastE(turtle).x + turtlesLastE(turtle).y
        val newE: Double = ke2 + pe2
        val eDiff: Double = newE - oldE
        val vFin: Double = StrictMath.sqrt(StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0) - (2 * eDiff / point.getBody2.getMass.getMass))
        if (vFin >= 0) {
          if ((StrictMath.abs(eDiff) > eDiffTolerance) && doConservation) {
            numCorrections += 1
            point.getBody2.getLinearVelocity.setMagnitude(vFin)
          }
        }
        val kef2: Double = 0.5 * point.getBody2.getMass.getMass * StrictMath.pow(point.getBody2.getLinearVelocity.getMagnitude, 2.0)
        val pef2: Double = (floor - point.getBody2.getTransform.getTranslationY) * world.getGravity.y * point.getBody2.getMass.getMass
        turtlesLastE.put(turtle, new Vector2(pef2, kef2))
        val newEf: Double = kef2 + pef2
        val eDiffF: Double = newEf - oldE
        if (StrictMath.abs(eDiffF) > eDiffTolerance)
        {
          totalEnergyDiscrepancy += newEf - oldE
          numEnergyDiscrepancies += 1
         // numWallDiscrepancies += 1
        }
      }
      val body1: Body = point.getBody1
      val body2: Body = point.getBody2
      if (bodiesToTurtles.contains(body1) && bodiesToTurtles.contains(body2)) { // Collision List Building

        val turtle1: Turtle = bodiesToTurtles(body1)
        val turtle2: Turtle = bodiesToTurtles(body2)
        turtlesCollisionList = turtlesCollisionList.concat(mutable.Map(turtle1 -> turtlesCollisionList.getOrElseUpdate(turtle1, List[Turtle]()).::(turtle2)))
        turtlesCollisionList = turtlesCollisionList.concat(mutable.Map(turtle2 -> turtlesCollisionList.getOrElseUpdate(turtle2, List[Turtle]()).::(turtle1)))

      } else if (bodiesToTurtles.contains(body1) && bodiesToPatches.contains(body2)) {

        val turtle: Turtle = bodiesToTurtles(body1)
        val patch: Patch = bodiesToPatches(body2)
        patchesCollisionList = patchesCollisionList.concat(mutable.Map(turtle -> patchesCollisionList.getOrElseUpdate(turtle, List[Patch]()).::(patch)))
        turtlesCollisionListPatches = turtlesCollisionListPatches.concat(mutable.Map(patch -> turtlesCollisionListPatches.getOrElseUpdate(patch, List[Turtle]()).::(turtle)))

      } else if (bodiesToPatches.contains(body1) && bodiesToTurtles.contains(body2)) {

        val patch: Patch = bodiesToPatches(body1)
        val turtle: Turtle = bodiesToTurtles(body2)
        patchesCollisionList = patchesCollisionList.concat(mutable.Map(turtle -> patchesCollisionList.getOrElseUpdate(turtle, List[Patch]()).::(patch)))
        turtlesCollisionListPatches = turtlesCollisionListPatches.concat(mutable.Map(patch -> turtlesCollisionListPatches.getOrElseUpdate(patch, List[Turtle]()).::(turtle)))

      }
      collisions += 1
      super.postSolve(point)
    }
  }

  object NumChanges extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {

      Double.box(turtleChanges)
    }


  }

  object TotalCorrections extends api.Reporter {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")
    override def report(args: Array[Argument], context: Context): AnyRef = {
      Double.box(numCorrections.toDouble)
    }
  }


  object TotalUncorrectable extends api.Reporter {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")
    override def report(args: Array[Argument], context: Context): AnyRef = {
      Double.box(numUncorrectable.toDouble)
    }


  }

  object EnergyDiscrepancy extends api.Reporter {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")
    override def report(args: Array[Argument], context: Context): AnyRef = {
      Double.box(totalEnergyDiscrepancy)
    }
  }

  object EnergyDiscrepancies extends api.Reporter {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")
    override def report(args: Array[Argument], context: Context): AnyRef = {
      Double.box(numEnergyDiscrepancies.toDouble)
    }
  }

  object GetMLC extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {

      Double.box(world.getSettings.getMaximumLinearCorrection)
    }


  }

  object TurtCols extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.AgentsetType, agentClassString = "-TP-")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      context.getAgent match {
        case turtle: Turtle => {
          var turtles: AgentSetBuilder = new AgentSetBuilder(AgentKind.Turtle)
          if (turtlesCollisionList.contains(turtle)) {
            turtlesCollisionList(turtle).foreach(turtle1 => if (!turtles.contains(turtle1)) {
              turtles.add(turtle1)
            })
          }
          turtles.build()
        }
        case patch: Patch => {
          var patches: AgentSetBuilder = new AgentSetBuilder(AgentKind.Turtle)
          if (turtlesCollisionListPatches.contains(patch)) {
            turtlesCollisionListPatches(patch).foreach(turtle1 => if (!patches.contains(turtle1)) {
              patches.add(turtle1)
            })
          }
          patches.build()
        }
      }
      }
    }

  object PatchCols extends api.Reporter {

    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.AgentsetType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {


          val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
          var patches: AgentSetBuilder = new AgentSetBuilder(AgentKind.Patch)
          if (patchesCollisionList.contains(turtle)) {
            patchesCollisionList(turtle).foreach(patch1 => if (!patches.contains(patch1)) {
              patches.add(patch1)
            })
          }
          patches.build()
        }

  }


  object CenterOfMass extends api.Reporter {

    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.ListType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      var com: LogoListBuilder = new LogoListBuilder()
      com.add(Double.box(turtlesToBodies(turtle).getMass.getCenter.x))
      com.add(Double.box(turtlesToBodies(turtle).getMass.getCenter.y))
      com.toLogoList
    }

  }

  object GetV extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
    val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
    Double.box(turtlesToBodies(turtle).getLinearVelocity.getMagnitude)
  }
  }

  object GetMass extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      Double.box(turtlesToBodies(turtle).getMass.getMass)
    }
  }

  object Inertia extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      Double.box(turtlesToBodies(turtle).getMass.getInertia)
    }
  }

  object GetKE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      Double.box(0.5 * turtlesToBodies(turtle).getMass.getMass * StrictMath.pow(turtlesToBodies(turtle).getLinearVelocity.getMagnitude, 2.0))
    }
  }

  object GetE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      Double.box(0.5 * turtlesToBodies(turtle).getMass.getMass * StrictMath.pow(turtlesToBodies(turtle).getLinearVelocity.getMagnitude, 2.0) + (floor - turtlesToBodies(turtle).getTransform.getTranslationY) * world.getGravity.y * turtlesToBodies(turtle).getMass.getMass)
    }
  }


  object GetVx extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      Double.box(turtlesToBodies(turtle).getLinearVelocity.x)
    }


  }

  object GetVy extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "-T--")

    override def report(args: Array[Argument], context: Context): AnyRef = {
      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      Double.box(turtlesToBodies(turtle).getLinearVelocity.y)
    }


  }

  object SetMass extends api.Command
  {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType), agentClassString = "-T--")
    override def perform(args: Array[Argument], context: Context): Unit = {

      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      turtlesToBodies(turtle).setMass(new Mass(new Vector2(0,0),args(0).getDoubleValue,0.0))

    }


  }


  object SetV extends api.Command
  {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType, Syntax.NumberType), agentClassString = "-T--")
    override def perform(args: Array[Argument], context: Context): Unit = {

      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      turtlesToBodies(turtle).setLinearVelocity(args(0).getDoubleValue, args(1).getDoubleValue)

    }


  }

  object SetVMag extends api.Command
  {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List(Syntax.NumberType), agentClassString = "-T--")
    override def perform(args: Array[Argument], context: Context): Unit = {

      val turtle: Turtle = context.getAgent.asInstanceOf[Turtle]
      turtlesToBodies(turtle).getLinearVelocity.setMagnitude(args(0).getDoubleValue)

    }


  }

  object ColNumber extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {

    Double.box(collisions)
  }


  }

  object TotalNewE extends api.Reporter
  {
    override def getSyntax: Syntax = Syntax.reporterSyntax(right = List(), ret = Syntax.NumberType, agentClassString = "O---")

    override def report(args: Array[Argument], context: Context): AnyRef = {

      Double.box(newE)
    }


  }

  object StopAll extends api.Command {
    override def getSyntax: Syntax = Syntax.commandSyntax(right = List())

    override def perform(args: Array[Argument], context: Context): Unit = {
      turtlesToBodies.foreach { case (turtle, body) => body.setLinearVelocity(new Vector2(0.0, 0.0))}
      }
  }
}
