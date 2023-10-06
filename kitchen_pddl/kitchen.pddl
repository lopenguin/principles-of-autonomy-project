(define (domain dwr)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    location  ; position in the world where items/robots can be
    robot     ;
    item      ; holds at most 1 container, only 1 robot per location
  )

  (:predicates
    (located ?i - item ?l - location)       ; item ?i at location ?l
    (holding ?i)
  )

  ; moves a robot between two adjacent locations
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (adjacent ?from ?to) (at ?r ?from) (not (occupied ?to)))
    :effect (and (at ?r ?to) (not (occupied ?from)) (occupied ?to) (not (at ?r ?from)))
  )

  ; loads an empty robot with a container held by a nearby crane
  (:action load
    :parameters (?k - crane ?r - robot ?c - container ?l - location)
    :precondition (and (at ?r ?l) (belong ?k ?l) (holding ?k ?c) (unloaded ?r) (not (equal ?c pallet)))
    :effect (and  (loaded ?r ?c) (not (unloaded ?r)) (empty ?k) (not (holding ?k ?c)))
  )

  ; unloads a robot holding a container with a nearby crane
  (:action unload
    :parameters (?k - crane ?r - robot ?c - container ?l - location)
    :precondition (and (belong ?k ?l) (at ?r ?l) (loaded ?r ?c) (empty ?k) (not (equal ?c pallet)))
    :effect (and (unloaded ?r) (holding ?k ?c) (not (loaded ?r ?c)) (not (empty ?k)))
  )

  ; puts a container held by a crane on a pile
  (:action put
    :parameters (?k - crane ?c - container ?c2 - container ?p - pile ?l - location)
    :precondition (and (belong ?k ?l) (attached ?p ?l) (holding ?k ?c) (top ?c2 ?p) (not (equal ?c ?c2)) (not (equal ?c pallet)))
    :effect (and (in ?c ?p) (top ?c ?p) (on ?c ?c2) (not (top ?c2 ?p)) (not (holding ?k ?c)) (empty ?k))
  )

  ; takes a container from a pile with a crane
  (:action take
    :parameters (?k - crane ?c - container ?c2 - container ?p - pile ?l - location)
    :precondition (and (belong ?k ?l) (attached ?p ?l) (empty ?k) (in ?c ?p) (top ?c ?p) (on ?c ?c2) (not (equal ?c ?c2)) (not (equal ?c pallet)))
    :effect (and (holding ?k ?c) (top ?c2 ?p) (not (in ?c ?p)) (not (top ?c ?p)) (not (on ?c ?c2)) (not (empty ?k)))
  )
)