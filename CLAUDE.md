# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Coal-based collision checking plugin for the Tesseract motion planning framework. Implements discrete and continuous (swept/cast) collision detection using the Coal library, as an alternative to the existing Bullet and FCL backends. Architecture closely mirrors Tesseract's Bullet collision implementation.

The reference Bullet and FCL implementations live in `src/tesseract/tesseract_collision/`.

## Coal fork requirement

Continuous collision detection requires `GEOM_CUSTOM` support in Coal for the `CastHullShape` class. The workspace's `src/coal/` fork provides this. Without it, Coal's narrowphase dispatcher won't recognize custom shapes.

## Plugin configuration

Two factory classes (`CoalDiscreteBVHManagerFactory`, `CoalCastBVHManagerFactory`) registered via macros in `coal_factories.cpp`. Optional YAML config keys:
- `gjk_guess_threshold` (both factories, default 5mm)
- `d_arc_compensation` (cast factory only, default false) — arc-sagitta swept-sphere inflation for rotational motions

## Test organization

1. **Test suite headers** (`test_suite/include/`) — reusable `.hpp` headers shared across collision backends.
2. **Integration tests** (`test/*.cpp`) — instantiate Coal managers and call test suite `runTest()` functions.
3. **Coal-specific unit tests** (`test/coal/`) — internal component tests (CastHullShape, caching, shape conversion, etc.).

## Commit conventions

Follow the existing style: imperative mood, concise subject line explaining the "why", body for details when needed.
