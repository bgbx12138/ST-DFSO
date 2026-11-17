"""Exact rows from existing frozen coefficients. No interpolation or new physics."""
import hashlib
from pathlib import Path
# Reviewed revision: only declared speed support changed; exact frequency row selection is unchanged.
HISTORICAL_ADAPTER_SHA256='2f16f8187eac01b286771550a041a2197b8d05c29fe2f381cf4a6138f46ace8c'
ORIGINAL_ADAPTER_SHA256='ed7846104a2699bc73ca6943af8bffc773a1bca0b5d97db7783c9506db84215f'

def exact_indices(grid,target,error_type):
    import numpy as np
    grid=np.asarray(grid);target=np.asarray(target)
    if (grid.ndim!=1 or target.ndim!=1 or len(grid)==0
            or not np.isfinite(grid).all() or not np.isfinite(target).all()
            or not np.all(np.diff(grid)>0)):
        raise error_type('Cross-input frozen frequency support differs; no extrapolation')
    ix=np.searchsorted(grid,target)
    if not np.all(ix<len(grid)) or not np.array_equal(grid[ix],target):
        raise error_type('Cross-input frozen frequency support differs; no extrapolation')
    return ix

def install(adapter):
    assert hashlib.sha256(Path(adapter.__file__).read_bytes()).hexdigest()==ORIGINAL_ADAPTER_SHA256,'ORIGINAL_ADAPTER_CHANGED392'
    if getattr(adapter._cross_coefficients,'_exact_frequency_index392',False):return
    original=adapter._cross_coefficients
    assert Path(original.__code__.co_filename).resolve()==Path(adapter.__file__).resolve(),'UNEXPECTED_CROSS_COEFFICIENT_WRAPPER392'
    def cross_coefficients(mu,w):
        import numpy as np
        e,_,_=adapter.engine();values=[]
        for node,weight in adapter.angular_weights(mu):
            if node not in adapter._CROSS:
                with np.load(adapter.cache_file(e.FOLDER/('cross_gauss_s2_mu%d.npz'%node))) as raw:
                    adapter._CROSS[node]=(raw['w'].copy(),raw['C'].copy())
            grid,coeff=adapter._CROSS[node]
            ix=exact_indices(grid,w,adapter.MissingFrozenInput)
            values.append(weight*coeff[ix])
        return sum(values)
    cross_coefficients._exact_frequency_index392=True
    adapter._cross_coefficients=cross_coefficients
