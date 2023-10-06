(define (domain dwr)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    location  ; position in the world where items/robots can be
    item      ; an object in the world that can be held.
  )

  (:predicates
    (located ?i - item ?l - location)       ; item ?i at location ?l
    (armAt ?l - location)        ; arm at location ?l
    (holding ?i - item)                 ; object being held by arm
    (drawerOpen)                 ; drawer open 
  )

  ; moves the arm tip from its current location to a desired one.
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and (armAt ?from))
    :effect (and (armAt ?to) (not (armAt ?from)))
  )

  ; causes the arm to grab the item if they are in the same location
  (:action grab
    :parameters (?item - item ?l - location)
    :precondition (and (armAt ?l) (located ?item ?l) )
    :effect (and (holding ?item) (not (located ?item ?l)))
  )

  ; places the object in the hand of the object
  (:action place
    :parameters (?item - item ?to - location)
    :precondition (and (armAt ?from))
    :effect (and (armAt ?to) (not (armAt ?from)))
  )

  ; moves the arm tip from its current location to a desired one.
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and (armAt ?from))
    :effect (and (armAt ?to) (not (armAt ?from)))
  )
)