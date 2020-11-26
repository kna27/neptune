//VARIABLE INIT
lock targetApoapsis to 80000.
lock targetTWR to 3.

//LAUNCH SEQUENCE
lock throttle to 1.
lock steering to UP + R(0,0,270).
print("3...").
wait 1.
print("2...").
wait 1.
safeStage().
print("1...").
wait 1.
safeStage().
print("Liftoff!").

//ASCENT SEQUENCE
lock steering to heading(90,90 * (1 - (ship:altitude / body:atm:height) ^ 0.5))+ R(0,0,270).
lock distance to SHIP:ALTITUDE + BODY("Kerbin"):RADIUS.
lock weight to CONSTANT():G * ((SHIP:MASS* BODY("Kerbin"):MASS) / ( distance * distance )).
lock THROTTLE to (targetTWR * weight) / (SHIP:MAXTHRUST + 0.001).
wait until ship:MAXTHRUST = 0.
print("MECO").

//STAGING AND BURN TO TARGET APOAPSIS
print("Staging...").
safeStage().
print("SES").
safeStage().
lock steering to ship:prograde.
wait until ship:apoapsis > targetApoapsis-2.
lock THROTTLE to 0.
print("SECO").

//CIRCULARIZATION MANEUVER
print("Calculating circularization...").
set targetV to sqrt(ship:body:mu/(ship:orbit:body:radius + ship:orbit:apoapsis)).
set apVel to sqrt(((1 - ship:orbit:ECCENTRICITY) * ship:orbit:body:mu) / ((1 + ship:orbit:ECCENTRICITY) * ship:orbit:SEMIMAJORAXIS)).
set dv to targetV - apVel.
set circnode to node(time:seconds + eta:apoapsis, 0, 0, dv).
add circnode. 
set np to circnode:deltav.
lock steering to np.
set max_acc to ship:maxthrust/ship:mass.
set burn_duration to circnode:deltav:mag/max_acc.
set tset to 0.
lock throttle to tset.
set done to False.
wait until ship:altitude > 51000.
safeStage().


//EXECUTE CIRCULARIZATION BURN
set dv0 to circnode:deltav.
wait until circnode:eta <= (burn_duration/2).
print("Circularizing...").
until done
{
    set max_acc to ship:maxthrust/ship:mass.
    set tset to min(circnode:deltav:mag/max_acc, 1).
    if vdot(dv0, circnode:deltav) < 0
    {
        lock throttle to 0.
        break.
    }
    if circnode:deltav:mag < 0.1
    {
        wait until vdot(dv0, circnode:deltav) < 0.5.
        lock throttle to 0.
        set done to True.
    }
}

//DEPLOY PAYLOAD AND CLEAN UP
print("SECO").
wait 5.
unlock steering.
unlock throttle.
remove circnode.
print("Exiting program.").

function SafeStage {
  wait until stage:ready.
  stage.
}