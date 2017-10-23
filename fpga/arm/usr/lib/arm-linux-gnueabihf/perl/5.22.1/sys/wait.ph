require '_h2ph_pre.ph';

no warnings qw(redefine misc);

unless(defined(&_SYS_WAIT_H)) {
    eval 'sub _SYS_WAIT_H () {1;}' unless defined(&_SYS_WAIT_H);
    require 'features.ph';
    require 'signal.ph';
    if(!defined (&_STDLIB_H) || (!defined (&__USE_XOPEN)  && !defined (&__USE_XOPEN2K8))) {
	require 'bits/waitflags.ph';
	if(defined(&__USE_MISC)) {
	    if(defined (&__GNUC__)  && !defined (&__cplusplus)) {
		eval 'sub __WAIT_INT {
		    my($status) = @_;
    		    eval q(( &__extension__ (((\'union union\' {  &__typeof($status)  &__in; \'int\'  &__i; }) { . &__in = ($status) }). &__i)));
		}' unless defined(&__WAIT_INT);
	    } else {
		eval 'sub __WAIT_INT {
		    my($status) = @_;
    		    eval q((*( &const \'int\' *) ($status)));
		}' unless defined(&__WAIT_INT);
	    }
	    if(!defined (&__GNUC__) || (defined(&__GNUC__) ? &__GNUC__ : undef) < 2|| defined (&__cplusplus)) {
		eval 'sub __WAIT_STATUS () { &void *;}' unless defined(&__WAIT_STATUS);
		eval 'sub __WAIT_STATUS_DEFN () { &void *;}' unless defined(&__WAIT_STATUS_DEFN);
	    } else {
		eval 'sub __WAIT_STATUS_DEFN () {\'int\' *;}' unless defined(&__WAIT_STATUS_DEFN);
	    }
	} else {
	    eval 'sub __WAIT_INT {
	        my($status) = @_;
    		eval q(($status));
	    }' unless defined(&__WAIT_INT);
	    eval 'sub __WAIT_STATUS () {\'int\' *;}' unless defined(&__WAIT_STATUS);
	    eval 'sub __WAIT_STATUS_DEFN () {\'int\' *;}' unless defined(&__WAIT_STATUS_DEFN);
	}
	require 'bits/waitstatus.ph';
	eval 'sub WEXITSTATUS {
	    my($status) = @_;
    	    eval q( &__WEXITSTATUS ( &__WAIT_INT ($status)));
	}' unless defined(&WEXITSTATUS);
	eval 'sub WTERMSIG {
	    my($status) = @_;
    	    eval q( &__WTERMSIG ( &__WAIT_INT ($status)));
	}' unless defined(&WTERMSIG);
	eval 'sub WSTOPSIG {
	    my($status) = @_;
    	    eval q( &__WSTOPSIG ( &__WAIT_INT ($status)));
	}' unless defined(&WSTOPSIG);
	eval 'sub WIFEXITED {
	    my($status) = @_;
    	    eval q( &__WIFEXITED ( &__WAIT_INT ($status)));
	}' unless defined(&WIFEXITED);
	eval 'sub WIFSIGNALED {
	    my($status) = @_;
    	    eval q( &__WIFSIGNALED ( &__WAIT_INT ($status)));
	}' unless defined(&WIFSIGNALED);
	eval 'sub WIFSTOPPED {
	    my($status) = @_;
    	    eval q( &__WIFSTOPPED ( &__WAIT_INT ($status)));
	}' unless defined(&WIFSTOPPED);
	if(defined(&__WIFCONTINUED)) {
	    eval 'sub WIFCONTINUED {
	        my($status) = @_;
    		eval q( &__WIFCONTINUED ( &__WAIT_INT ($status)));
	    }' unless defined(&WIFCONTINUED);
	}
    }
    if(defined(&__USE_MISC)) {
	eval 'sub WCOREFLAG () { &__WCOREFLAG;}' unless defined(&WCOREFLAG);
	eval 'sub WCOREDUMP {
	    my($status) = @_;
    	    eval q( &__WCOREDUMP ( &__WAIT_INT ($status)));
	}' unless defined(&WCOREDUMP);
	eval 'sub W_EXITCODE {
	    my($ret, $sig) = @_;
    	    eval q( &__W_EXITCODE ($ret, $sig));
	}' unless defined(&W_EXITCODE);
	eval 'sub W_STOPCODE {
	    my($sig) = @_;
    	    eval q( &__W_STOPCODE ($sig));
	}' unless defined(&W_STOPCODE);
    }
    if(defined(&__USE_MISC)) {
	eval 'sub WAIT_ANY () {(-1);}' unless defined(&WAIT_ANY);
	eval 'sub WAIT_MYPGRP () {0;}' unless defined(&WAIT_MYPGRP);
    }
    if(defined (&__USE_XOPEN) || defined (&__USE_XOPEN2K8)) {
	unless(defined(&__id_t_defined)) {
	    require 'bits/types.ph';
	    eval 'sub __id_t_defined () {1;}' unless defined(&__id_t_defined);
	}
	eval 'sub __need_siginfo_t () {1;}' unless defined(&__need_siginfo_t);
	require 'bits/siginfo.ph';
    }
    if(defined (&__USE_MISC) || defined (&__USE_XOPEN_EXTENDED)) {
    }
    if(defined(&__USE_MISC)) {
    }
}
1;
