(define (problem problem)
  (:domain kitchen)

  (:objects

    stove countertop - surface
    drawer - openable
    sugar spam - object

  )
  (:init (and
    (objectOn sugar stove)
    (objectOn spam countertop)
    )
  )
  (:goal (and
    (objectOn sugar countertop)
    (objectIn spam drawer)
  ))
)