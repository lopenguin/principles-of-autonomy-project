(define (problem problem1)
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
    (boxOn sugar countertop)
    (boxIn spam drawer)
  ))
)