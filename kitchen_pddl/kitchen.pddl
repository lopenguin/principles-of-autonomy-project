(define (domain kitchen)
  (:requirements :strips :typing :negative-preconditions :universal-preconditions)

  (:types
    surface   ; can be placed on without opening
    openable  ; must be opened before placing
    object    ; item that can be moved
  )

  (:predicates
    (holding ?obj - object)
    (objectOn ?obj - object ?s - surface)
    (opened ?o - openable)
    (objectIn ?obj - object ?o - openable)
  )

  ; places an object onto a surface
  (:action placeOn
    :parameters (?obj - object ?s - surface)
    :precondition (and (holding ?obj)
                      (forall (?allObjs - object)
                        (not (objectOn ?allObjs ?s))
                      )
                  )
    :effect (and (objectOn ?obj ?s) (not (holding ?obj)))
  )

  ; places an object into an open cabinet
  (:action placeIn
    :parameters (?obj - object ?o - openable)
    :precondition (and (holding ?obj) (opened ?o)
                      (forall (?allObjs - object)
                        (not (objectIn ?allObjs ?o))
                      )
                  )
    :effect (and (objectIn ?obj ?o) (not (holding ?obj)))
  )

  ; opens a cabinet
  (:action open
    :parameters (?o - openable)
    :precondition (and (not (opened ?o))
                      (forall (?allObjs - object)
                        (not (holding ?allObjs))
                      )
                  )
    :effect (and (opened ?o))
  )

  ; pick up an object (from a surface)
  (:action pickUp
    :parameters (?obj - object ?s - surface)
    :precondition (and (objectOn ?obj ?s)
                      (forall (?allObjs - object)
                        (not (holding ?allObjs))
                      )
                  )
    :effect (and (holding ?obj) (not (objectOn ?obj ?s)))
  )

  ; pick up an object (from a cabinet)
  (:action cabPickUp
    :parameters (?obj - object ?o - openable)
    :precondition (and (objectOn ?obj ?o) (opened ?o)
                      (forall (?allObjs - object)
                        (not (holding ?allObjs))
                      )
                  )
    :effect (and (holding ?obj) (not (objectOn ?obj ?o)))
  )
)