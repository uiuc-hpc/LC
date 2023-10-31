from spack import *

class Lci(CMakePackage):
    """LCI: the Lightweight Communication Interface"""

    homepage = "https://github.com/uiuc-hpc/LC"
    url      = "https://github.com/uiuc-hpc/LC/archive/v2.zip"
    git      = "git@github.com:uiuc-hpc/LC.git"

    version('v2', branch='v2-cmake-tmp')

    def is_positive_int(val):
        try:
            return int(val) > 0
        except ValueError:
            return val == 'auto'

    def is_nonnegative_int(val):
        try:
            return int(val) >= 0
        except ValueError:
            return val == 'auto'

    variant('ipo', default=False, description='Interprocedural Optimization')
    variant('fabric', default='ibv',
            values=('ofi', 'ibv', 'psm'), multi=False,
            description='Communication fabric')
    variant('dreg', default=True, description='Use registration cache')
    variant('addressing', default='dynamic,explicit',
            values=('dynamic', 'explicit', 'immediate'), multi=True,
            description='Endpoint addressing mode')
    variant('completion', default='sync,cq',
            values=('sync', 'cq', 'am', 'glob'), multi=True,
            description='Completion mechanism')
    variant('devices', default='1', values=is_positive_int,
            description='Number of NICs')
    variant('endpoints', default='8', values=is_positive_int,
            description='Number of endpoints')
    variant('max_rcvs', default='auto', values=is_positive_int,
            description='Max posted recieves')
    variant('packets', default='auto', values=is_positive_int,
            description='Number of packets')
    variant('med_pkt_ret_sz', default='auto', values=is_nonnegative_int,
            description='Min size of medium packet to return to sender pool')
    variant('long_pkt_ret', default=False,
            description='Return long packets to sender pool')

    depends_on('cmake@3.13:', type='build')
    depends_on('libfabric', when='fabric=ofi')
    depends_on('rdma-core', when='fabric=ibv')
    depends_on('opa-psm2',  when='fabric=psm')

    def cmake_args(self):
        args = [
            self.define_from_variant('CMAKE_INTERPROCEDURAL_OPTIMIZATION', 'ipo'),
            self.define_from_variant('LC_SERVER', 'fabric'),
            self.define_from_variant('LC_USE_DREG', 'dreg'),
            self.define_from_variant('LC_EP_AR', 'addressing'),
            self.define_from_variant('LC_EP_CE', 'completion'),
            self.define_from_variant('LC_MAX_DEV', 'devices'),
            self.define_from_variant('LC_MAX_EP', 'endpoints')
        ]

        if self.spec.variants['max_rcvs'].value != 'auto':
            arg = self.define_from_variant('LC_SERVER_MAX_RCVS', 'max_rcvs')
            args.append(arg)

        if self.spec.variants['packets'].value != 'auto':
            arg = self.define_from_variant('LC_SERVER_NUM_PKTS', 'packets')
            args.append(arg)

        if self.spec.variants['med_pkt_ret_sz'].value != 'auto':
            arg = self.define_from_variant('LC_PKT_RET_MED_SIZE', 'med_pkt_ret_sz')
            args.append(arg)

        if self.spec.variants['long_pkt_ret'].value != 'auto':
            arg = self.define_from_variant('LC_PKT_RET_LONG', 'long_pkt_ret')
            args.append(arg)

        return args
