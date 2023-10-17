(define (problem problem1)
  (:domain kitchen)

  (:objects

    stove countertop - surface
    drawer - openable
    sugar spam - object

  )
  (:init
    (objectOn ?sugar - object ?stove - surface)
    (objectOn ?spam - object ?countertop - surface)
  )
  (:goal (and
    (objectOn ?sugar - object ?countertop - surface)
    (objectIn ?spam - object ?drawer - openable)
  ))
)