(define (domain dwr)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    location  ; position in the world where items/robots can be
    item      ; an object in the world that can be held.
  )

  (:predicates
    (located ?i - item ?l - location)   ; item ?i at location ?l (not in a drawer)
    (armAt ?l - location)               ; arm at location ?l
    (holding ?i - item)                 ; object being held by arm
    (drawerOpen)                        ; drawer open 
  )

    ; TODO: THINK A LOT MORE ABOUT THESE!

  ; moves the arm tip from its current location to a desired one.
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and (armAt ?from))
    :effect (and (armAt ?to) (not (armAt ?from)))
  )

  ; causes the arm to grab the item if they are in the same location
  (:action grab
    :parameters (?i - item ?l - location)
    :precondition (and (armAt ?l) (located ?i ?l) )
    :effect (and (holding ?i) (not (located ?i ?l)))
  )

  ; places the object on top of something
  (:action place
    :parameters (?i - item ?l - location)
    :precondition (and (armAt ?l) (holding ?i))
    :effect (and (located ?i ?l) (not (holding ?i)))
  )

  ; moves the arm tip from its current location to a desired one.
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and (armAt ?from))
    :effect (and (armAt ?to) (not (armAt ?from)))
  )
)