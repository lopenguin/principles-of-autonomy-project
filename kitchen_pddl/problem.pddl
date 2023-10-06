(define (problem problem1)
  (:domain kitchen)

  (:objects

    stove countertop drawer_closed drawer_open - location
    sugar spam - item

  )
  (:init
    (located ?sugar - item ?stove - location)
    (located ?spam - item ?countertop - location)
    (armAt ?stove - location)
  )
  (:goal (and
    (located ?sugar - item ?countertop - location)
    (located ?spam - item ?drawer_closed - location)
  ))
)