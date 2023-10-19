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

  ; ; first step only
  ; (:goal (and
  ;   (boxOn sugar stove)
  ;   (boxOn spam countertop)
  ;   (armClear)
  ;   (cabClear drawer)
  ;   (opened drawer)
  ; ))

  ; ; up to second step
  ; (:goal (and
  ;   (boxOn sugar stove)
  ;   (not (boxOn spam countertop))
  ;   (not (armClear))
  ;   (holding spam)
  ;   (cabClear drawer)
  ;   (opened drawer)
  ; ))

  ; ; up to third step
  ; (:goal (and
  ;   (boxOn sugar stove)
  ;   (not (boxOn spam countertop))
  ;   (armClear)
  ;   (not (holding spam))
  ;   (not (cabClear drawer))
  ;   (boxIn spam drawer)
  ;   (opened drawer)
  ; ))

  ; ; up to fourth step
  ; (:goal (and
  ;   (not (boxOn sugar stove))
  ;   (not (boxOn spam countertop))
  ;   (not (armClear))
  ;   (not (holding spam))
  ;   (not (cabClear drawer))
  ;   (boxIn spam drawer)
  ;   (opened drawer)
  ; ))

  ; up to fifth step
  (:goal (and
    (not (boxOn sugar stove))
    (not (boxOn spam countertop))
    (armClear)
    (not (holding spam))
    (not (holding sugar))
    (not (cabClear drawer))
    (boxIn spam drawer)
    (opened drawer)
    (boxOn sugar countertop)
  ))
)