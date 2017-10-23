# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from xml.dom import minidom

from rosdistro import logger


def sanitize_xml(xml_string):
    """ Returns a version of the supplied XML string with comments and all whitespace stripped,
    including runs of spaces internal to text nodes. The returned string will be encoded,
    so str (Python 2) or bytes (Python 3).
    """
    def _squash(node):
        drop_nodes = []
        for x in node.childNodes:
            if x.nodeType == minidom.Node.TEXT_NODE:
                if x.nodeValue:
                    x.nodeValue = ' '.join(x.nodeValue.strip().split())
            elif x.nodeType == minidom.Node.ELEMENT_NODE:
                _squash(x)
            elif x.nodeType is minidom.Node.COMMENT_NODE:
                drop_nodes.append(x)
        for x in drop_nodes:
            node.removeChild(x)
        return node
    try:
        # Python 2. The minidom module parses as ascii, so we have to pre-encode.
        if isinstance(xml_string, unicode):
            xml_string = xml_string.encode('utf-8')
    except NameError:
        # Python 3. Strings are native unicode.
        pass

    xml_node = _squash(minidom.parseString(xml_string))
    try:
        # Python 2. Encode the resultant XML as a str.
        unicode
        return xml_node.toxml('utf-8')
    except NameError:
        # Python 3. Return native bytes.
        return xml_node.toxml()


class CachedManifestProvider(object):

    def __init__(self, distribution_cache, manifest_providers=None):
        self._distribution_cache = distribution_cache
        self._manifest_providers = manifest_providers

    def __call__(self, dist_name, repo, pkg_name):
        assert repo.version
        package_xml = self._distribution_cache.release_package_xmls.get(pkg_name, None)
        if package_xml:
            package_xml = sanitize_xml(package_xml)
            self._distribution_cache.release_package_xmls[pkg_name] = package_xml
            logger.debug('Loading package.xml for package "%s" from cache' % pkg_name)
        else:
            # use manifest providers to lazy load
            for mp in self._manifest_providers or []:
                try:
                    package_xml = sanitize_xml(mp(dist_name, repo, pkg_name))
                    break
                except Exception as e:
                    # pass and try next manifest provider
                    logger.debug('Skipped "%s()": %s' % (mp.__name__, e))
            if package_xml is None:
                return None
            # populate the cache
            self._distribution_cache.release_package_xmls[pkg_name] = package_xml
        return package_xml


class CachedSourceManifestProvider(object):

    def __init__(self, distribution_cache, source_manifest_providers=None):
        self._distribution_cache = distribution_cache
        self._source_manifest_providers = source_manifest_providers

    def __call__(self, repo):
        assert repo.url
        repo_cache = self._distribution_cache.source_repo_package_xmls.get(repo.name, None)
        if not repo_cache:
            # Use manifest providers to lazy load
            for mp in self._source_manifest_providers or []:
                try:
                    repo_cache = mp(repo)
                except Exception as e:
                    # pass and try next manifest provider
                    logger.debug('Skipped "%s()": %s' % (mp.__name__, e))
                    continue

                self._distribution_cache.source_repo_package_xmls[repo.name] = repo_cache
                break
        else:
            logger.debug('Load package XMLs for repo "%s" from cache' % repo.name)

        # De-duplicate with the release package XMLs. This will cause the YAML writer
        # to use references, saving a lot of space in the cache file.
        for key in repo_cache or {}:
            if key[0] != '_':
                repo_cache[key][1] = sanitize_xml(repo_cache[key][1])
                if key in self._distribution_cache.release_package_xmls:
                    release_package_xml = self._distribution_cache.release_package_xmls[key]
                    if repo_cache[key][1] == release_package_xml:
                        repo_cache[key][1] = release_package_xml

        return repo_cache
