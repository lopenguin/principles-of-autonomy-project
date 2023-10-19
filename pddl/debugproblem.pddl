(define (problem debug)
  (:domain kitchen)

  (:objects

    stove countertop - surface
    drawer - openable
    sugar spam - box

  )
  (:init 
    (boxOn sugar stove)
    (boxOn spam countertop)
    (armClear)
    (cabClear drawer)
  )
  (:goal (and
    (not (boxOn sugar stove))
    (boxOn spam countertop)
    (not (armClear))
    (cabClear drawer)
  ))
)