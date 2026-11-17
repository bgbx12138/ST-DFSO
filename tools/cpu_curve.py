# -*- coding: utf-8 -*-
"""Conservative direct QPC/Java11 ledger join; never consult UTC eligibility."""
import math

def ticks_ns(value, freq):
    # Integer conversion plus the existing 2ms guard covers JVM double rounding and one QPC tick.
    return value * 1000000000 // freq

def bracket(rows, lower, upper):
    assert lower <= upper
    left = [r for r in rows if ticks_ns(r['qpc_after_ticks'], r['qpc_frequency']) <= lower]
    right = [r for r in rows if ticks_ns(r['qpc_before_ticks'], r['qpc_frequency']) >= upper]
    if not left or not right:
        raise ValueError('QPC_OUTSIDE_OBSERVED_JOB_TIMELINE180')
    return left[-1]['process_tree_cpu_seconds'], right[0]['process_tree_cpu_seconds']

def validate_timeline(clock, rows, receipt, guard_ns=2000000):
    assert guard_ns == 2000000, 'REGISTERED_GUARD_CHANGED180'
    assert receipt['observer_error'] is None and receipt['complete_job_accounting']
    assert receipt['returncode'] == 0 and not receipt['timed_out']
    assert len(rows) == receipt['samples'] and len(rows) >= 2
    assert clock['pid'] == receipt['pid'] and clock['search_started']
    assert isinstance(clock['ledger_start_nano'], int) and clock['ledger_start_nano'] > 0
    assert isinstance(clock['ledger_elapsed_nanos'], int) and clock['ledger_elapsed_nanos'] >= 0
    assert clock['before']['nano_after'] <= clock['ledger_start_nano'] <= clock['after']['nano_before']
    freq = receipt['qpc_frequency']; assert isinstance(freq, int) and freq > 0
    assert rows[0]['phase'] == 'suspended_before_start' and rows[-1]['phase'] == 'final'
    assert rows[-1]['active'] == 0
    previous = None
    for i, row in enumerate(rows):
        assert row['index'] == i and row['root_pid'] == receipt['pid']
        assert row['qpc_frequency'] == freq
        assert isinstance(row['qpc_before_ticks'], int) and isinstance(row['qpc_after_ticks'], int)
        assert row['qpc_before_ticks'] <= row['qpc_after_ticks']
        cpu = row['process_tree_cpu_seconds']; assert math.isfinite(cpu) and cpu >= 0
        if previous is not None:
            assert row['qpc_before_ticks'] >= previous['qpc_after_ticks'], 'QPC_NONMONOTONE180'
            assert cpu >= previous['process_tree_cpu_seconds'], 'JOB_CPU_NONMONOTONE180'
        previous = row
    assert rows[-1]['process_tree_cpu_seconds'] == receipt['process_tree_cpu_seconds']
    start = clock['ledger_start_nano']
    bracket(rows, start - guard_ns, start + guard_ns)
    bracket(rows, clock['before']['nano_before'] - guard_ns, clock['before']['nano_after'] + guard_ns)
    bracket(rows, clock['after']['nano_before'] - guard_ns, clock['after']['nano_after'] + guard_ns)

def curve(clock, rows, milestones, receipt, final_cost, guard_ns=2000000):
    validate_timeline(clock, rows, receipt, guard_ns)
    assert milestones and math.isfinite(final_cost) and final_cost > 0
    start = clock['ledger_start_nano']; base = bracket(rows, start - guard_ns, start + guard_ns)
    points = []; search = []; previous_ns = -1; previous_work = -1; previous_cost = float('inf')
    for m in milestones:
        ns = m['wall_nanos']; work = m['subsegments']; cost = m['cost']
        assert isinstance(ns, int) and previous_ns <= ns <= clock['ledger_elapsed_nanos'] and ns >= 0
        assert isinstance(work, int) and work >= previous_work and work >= 0
        assert math.isfinite(cost) and 0 < cost <= previous_cost + 1e-8 * max(1, abs(previous_cost))
        lo, hi = bracket(rows, start + ns - guard_ns, start + ns + guard_ns)
        assert 0 <= lo <= hi <= receipt['process_tree_cpu_seconds']
        points.append(dict(m, full_job_cpu_lower_seconds=lo, full_job_cpu_upper_seconds=hi,
                           observation_scope='Direct Java11 ledger/QPC; CPU query enclosed before/after. Full child Job since suspended launch. No interpolation.'))
        search.append(dict(m, search_process_tree_cpu_lower_seconds=max(0., lo - base[1]),
                           search_process_tree_cpu_upper_seconds=max(0., hi - base[0])))
        previous_ns, previous_work, previous_cost = ns, work, cost
    points.append(dict(cost=final_cost, subsegments=milestones[-1]['subsegments'], checkpoint='validated_process_completion',
                       full_job_cpu_lower_seconds=receipt['process_tree_cpu_seconds'],
                       full_job_cpu_upper_seconds=receipt['process_tree_cpu_seconds'],
                       observation_scope='All full-Job descendants, validation and report CPU included. Finished incumbent persists.'))
    return dict(schema='FULL_JOB_AND_SEARCH_QPC_QUALITY180_V1', clock_pid=clock['pid'],
                qpc_frequency=receipt['qpc_frequency'], guard_ns=guard_ns,
                full_job_points=points, search_points=search, qpc_validated=True,
                original_utc_clock_consistent=clock['clock_consistent'],
                original_utc_join_eligible=clock['join_eligible'],
                seed_evaluation_included_in_full_job_cpu=True,
                scope='UTC eligibility preserved as evidence only. Independent same-source QPC gate. Main endpoint remains full child Job CPU; parent/observer are separate costs.')
