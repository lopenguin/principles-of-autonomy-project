(define (domain kitchen)
  (:requirements :strips :typing :negative-preconditions :universal-preconditions)

  (:types
    surface   ; can be placed on without opening
    openable  ; must be opened before placing
    box    ; item that can be moved
  )

  (:predicates
    (holding ?box - box)
    (boxOn ?box - box ?s - surface)
    (opened ?o - openable)
    (boxIn ?box - box ?o - openable)
  )

  ; places an box onto a surface
  (:action placeOn
    :parameters (?box - box ?s - surface)
    :precondition (and (holding ?box)
                  )
    :effect (and (boxOn ?box ?s) (not (holding ?box)))
  )

  ; places an box into an open cabinet
  (:action placeIn
    :parameters (?box - box ?o - openable)
    :precondition (and (holding ?box) (opened ?o)
                      (forall (?allboxs - box)
                        (not (boxIn ?allboxs ?o))
                      )
                  )
    :effect (and (boxIn ?box ?o) (not (holding ?box)))
  )

  ; opens a cabinet
  (:action open
    :parameters (?o - openable)
    :precondition (and (not (opened ?o))
                      (forall (?allboxs - box)
                        (not (holding ?allboxs))
                      )
                  )
    :effect (and (opened ?o))
  )

  ; pick up an box (from a surface)
  (:action pickUp
    :parameters (?box - box ?s - surface)
    :precondition (and (boxOn ?box ?s)
                      (forall (?allboxs - box)
                        (not (holding ?allboxs))
                      )
                  )
    :effect (and (holding ?box) (not (boxOn ?box ?s)))
  )

  ; pick up an box (from a cabinet)
  (:action cabPickUp
    :parameters (?box - box ?o - openable)
    :precondition (and (boxOn ?box ?o) (opened ?o)
                      (forall (?allboxs - box)
                        (not (holding ?allboxs))
                      )
                  )
    :effect (and (holding ?box) (not (boxOn ?box ?o)))
  )
)