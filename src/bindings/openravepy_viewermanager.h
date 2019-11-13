// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVEPY_INTERNAL_VIEWERMANAGER_H
#define OPENRAVEPY_INTERNAL_VIEWERMANAGER_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

/// \brief manages all the viewers created through SetViewer into a single thread
class ViewerManager
{
    /// \brief info about the viewer to create or that is created
    struct ViewerInfo
    {
        EnvironmentBasePtr _penv;
        std::string _viewername;
        ViewerBasePtr _pviewer; /// the created viewer
        boost::condition _cond;  ///< notify when viewer thread is done processing and has initialized _pviewer
        bool _bShowViewer; ///< true if should show the viewer when initially created
    };
    typedef OPENRAVE_SHARED_PTR<ViewerInfo> ViewerInfoPtr;
public:
    ViewerManager() {
        _bShutdown = false;
        _bInMain = false;
        _threadviewer.reset(new boost::thread(boost::bind(&ViewerManager::_RunViewerThread, this)));
    }

    virtual ~ViewerManager() {
        Destroy();
    }

    static ViewerManager& GetInstance() {
        boost::call_once(_InitializeSingleton, _onceInitialize);
        // Return reference to object.
        return *_singleton;
    }

    /// \brief adds a viewer to the environment whose GUI thread will be managed by _RunViewerThread
    ///
    /// \param bDoNotAddIfExists if true, will not add a viewer if one already exists and is added to the manager
    ViewerBasePtr AddViewer(EnvironmentBasePtr penv, const string &strviewer, bool bShowViewer, bool bDoNotAddIfExists=true)
    {
        ViewerBasePtr pviewer;
        if( strviewer.size() > 0 ) {

            if( bDoNotAddIfExists ) {
                // check all existing viewers
                boost::mutex::scoped_lock lock(_mutexViewer);
                std::list<ViewerInfoPtr>::iterator itviewer = _listviewerinfos.begin();
                while(itviewer != _listviewerinfos.end() ) {
                    if( (*itviewer)->_penv == penv ) {
                        if( (*itviewer)->_viewername == strviewer ) {
                            if( !!(*itviewer)->_pviewer ) {
                                (*itviewer)->_pviewer->Show(bShowViewer);
                            }
                            return (*itviewer)->_pviewer;
                        }

                        // should remove the viewer so can re-add a new one
                        if( !!(*itviewer)->_pviewer ) {
                            (*itviewer)->_penv->Remove((*itviewer)->_pviewer);
                        }
                        itviewer = _listviewerinfos.erase(itviewer);
                    }
                    else {
                        ++itviewer;
                    }
                }
            }

            ViewerInfoPtr pinfo(new ViewerInfo());
            pinfo->_penv = penv;
            pinfo->_viewername = strviewer;
            pinfo->_bShowViewer = bShowViewer;
            if( _bInMain ) {
                // create in this thread since viewer thread is already waiting on another viewer
                pviewer = RaveCreateViewer(penv, strviewer);
                if( !!pviewer ) {
                    penv->AddViewer(pviewer);
                    // TODO uncomment once Show posts to queue
                    if( bShowViewer ) {
                        pviewer->Show(1);
                    }
                    pinfo->_pviewer = pviewer;
                    boost::mutex::scoped_lock lock(_mutexViewer);
                    _listviewerinfos.push_back(pinfo);
                    _conditionViewer.notify_all();
                }
            }
            else {
                // no viewer has been created yet, so let the viewer thread create it (if using Qt, this initializes the QApplication in the right thread
                boost::mutex::scoped_lock lock(_mutexViewer);
                _listviewerinfos.push_back(pinfo);
                _conditionViewer.notify_all();

                /// wait until viewer thread process it
                pinfo->_cond.wait(_mutexViewer);
                pviewer = pinfo->_pviewer;
            }
        }
        return pviewer;
    }

    /// \brief if removed, returns true
    bool RemoveViewer(ViewerBasePtr pviewer)
    {
        if( !pviewer ) {
            return false;
        }
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            FOREACH(itviewer, _listviewerinfos) {
                ViewerBasePtr ptestviewer = (*itviewer)->_pviewer;
                if(ptestviewer == pviewer ) {
                    pviewer->quitmainloop();
                    _listviewerinfos.erase(itviewer);
                    return true;
                }
            }
        }
        return false;
    }

    /// \brief if anything removed, returns true
    bool RemoveViewersOfEnvironment(EnvironmentBasePtr penv)
    {
        if( !penv ) {
            return false;
        }
        bool bremoved = false;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            std::list<ViewerInfoPtr>::iterator itinfo = _listviewerinfos.begin();
            while(itinfo != _listviewerinfos.end() ) {
                if( (*itinfo)->_penv == penv ) {
                    itinfo = _listviewerinfos.erase(itinfo);
                    bremoved = true;
                }
                else {
                    ++itinfo;
                }
            }
        }
        return bremoved;
    }

    void Destroy() {
        _bShutdown = true;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            // have to notify everyone
            FOREACH(itinfo, _listviewerinfos) {
                (*itinfo)->_cond.notify_all();
            }
            _listviewerinfos.clear();
            _conditionViewer.notify_all();
        }
        if( !!_threadviewer ) {
            _threadviewer->join();
        }
        _threadviewer.reset();
    }

protected:
    void _RunViewerThread()
    {
        while(!_bShutdown) {
            std::list<ViewerBasePtr> listviewers, listtempviewers;
            bool bShowViewer = true;
            {
                boost::mutex::scoped_lock lock(_mutexViewer);
                if( _listviewerinfos.size() == 0 ) {
                    _conditionViewer.wait(lock);
                    if( _listviewerinfos.size() == 0 ) {
                        continue;
                    }
                }

                listtempviewers.clear(); // viewers to add to env once lock is released
                listviewers.clear();
                std::list<ViewerInfoPtr>::iterator itinfo = _listviewerinfos.begin();
                while(itinfo != _listviewerinfos.end() ) {
                    ViewerInfoPtr pinfo = *itinfo;
                    if( !pinfo->_pviewer ) {
                        pinfo->_pviewer = RaveCreateViewer(pinfo->_penv, pinfo->_viewername);
                        // have to notify other thread that viewer is present before the environment lock happens! otherwise we can get into deadlock between c++ and python
                        pinfo->_cond.notify_all();
                        if( !!pinfo->_pviewer ) {
                            listtempviewers.push_back(pinfo->_pviewer);
                            ++itinfo;
                        }
                        else {
                            // erase from _listviewerinfos
                            itinfo = _listviewerinfos.erase(itinfo);
                        }
                    }
                    else {
                        ++itinfo;
                    }

                    if( !!pinfo->_pviewer ) {
                        if( listviewers.size() == 0 ) {
                            bShowViewer = pinfo->_bShowViewer;
                        }
                        listviewers.push_back(pinfo->_pviewer);
                    }
                }
            }

            FOREACH(itaddviewer, listtempviewers) {
                (*itaddviewer)->GetEnv()->AddViewer(*itaddviewer);
            }

            ViewerBasePtr puseviewer;
            FOREACH(itviewer, listviewers) {
                // double check if viewer is added to env
                bool bfound = false;
                listtempviewers.clear();
                (*itviewer)->GetEnv()->GetViewers(listtempviewers);
                FOREACH(itviewer2, listtempviewers) {
                    if( *itviewer == *itviewer2 ) {
                        bfound = true;
                        break;
                    }
                }
                if( bfound ) {
                    puseviewer = *itviewer;
                    break;
                }
                else {
                    // viewer is not in environment any more, so erase from list
                    listviewers.erase(itviewer);
                    break; // break since modifying list
                }
            }

            listtempviewers.clear();

            if( !!puseviewer ) {
                _bInMain = true;
                try {
                    puseviewer->main(bShowViewer);
                }
                catch(const std::exception& ex) {
                    RAVELOG_ERROR_FORMAT("got exception in viewer main thread %s", ex.what());
                }
                catch(...) {
                    RAVELOG_ERROR("got unknown exception in viewer main thread\n");
                }

                _bInMain = false;
                // remove from _listviewerinfos in order to avoid running the main loop again
                {
                    boost::mutex::scoped_lock lock(_mutexViewer);
                    FOREACH(itinfo, _listviewerinfos) {
                        if( (*itinfo)->_pviewer == puseviewer ) {
                            _listviewerinfos.erase(itinfo);
                            break;
                        }
                    }
                }
                puseviewer.reset();
            }
            // just go and run the next viewer's loop, don't exit here!
        }
        RAVELOG_DEBUG("shutting down viewer manager thread\n");
    }

    static void _InitializeSingleton()
    {
        _singleton.reset(new ViewerManager());

    }

    OPENRAVE_SHARED_PTR<boost::thread> _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    std::list<ViewerInfoPtr> _listviewerinfos;

    bool _bShutdown; ///< if true, shutdown everything
    bool _bInMain; ///< if true, viewer thread is running a main function

    static boost::scoped_ptr<ViewerManager> _singleton; ///< singleton
    static boost::once_flag _onceInitialize; ///< makes sure initialization is atomic

}; // class ViewerManager

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_VIEWERMANAGER_H
